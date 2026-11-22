/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)

#include "common/maths.h"
#include "common/printf.h"
#include "common/time.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/dispatch.h"
#include "fc/runtime_config.h"

#include "flight/flight_plan_nav.h"

#include "io/gps.h"

#include "pg/flight_plan.h"
#include "pg/flight_plan_capture.h"

#include "flight_plan_capture.h"

// The gesture commits on RELEASE so a hold can mean delete; the position is
// sampled at the PRESS, so the moment the pilot marks is the point that is
// stored, not wherever the craft has drifted to by the release.
#define FP_CAPTURE_HOLD_DELETE_US 1500000u
#define FP_CAPTURE_MSG_TIME_US    2500000u

// AUTO recording. A turn is only judged once the craft is this far from the
// last point, so a metre or two of GPS wander sideways is not a turn.
#define FP_REC_TURN_MIN_CM        300.0f
// The end point is only added when it is this far from the last point.
#define FP_REC_END_MIN_CM         200.0f
#define FP_REC_END_MIN_ALT_CM     100
// Thinning weighs a height error as this many horizontal centimetres per cm,
// so a dropped climb counts as much as a dropped corner of the same size.
#define FP_REC_THIN_ALT_WEIGHT    1.0f

// Let disarming finish before the flash write stalls the scheduler, as the
// flight-stats save does.
#define FP_CAPTURE_SAVE_DELAY_US  500000

// 1e-7 degree of latitude in centimetres.
#define FP_CM_PER_E7_DEG          1.113195f

static bool switchWas = false;
static timeUs_t pressTimeUs = 0;
static bool deleteFired = false;      // this press already consumed as a delete
static waypoint_t pending;            // sampled at press, committed at release
static bool pendingValid = false;
static bool pressAccepted = false;    // press happened somewhere edits are allowed

// AUTO recording state
static bool recording = false;        // a route is being recorded
static bool pressConsumed = false;    // this switch-on already started (or was refused) a route

// Persisting the plan
static bool planUnsaved = false;      // the plan was edited since the last flash write

static char msgText[16];
static timeUs_t msgExpireUs = 0;
static bool msgValid = false;

static void setMessage(timeUs_t currentTimeUs, const char *fmt, int index)
{
    tfp_sprintf(msgText, fmt, index);
    msgExpireUs = currentTimeUs + FP_CAPTURE_MSG_TIME_US;
    msgValid = true;
}

// Edits are refused while the executor owns the plan: appending behind a
// flying mission or deleting the leg being flown would corrupt its index.
static bool editsAllowed(void)
{
    return !flightPlanNavIsActive();
}

static waypoint_t currentPositionWaypoint(void)
{
    return (waypoint_t){
        .latitude = gpsSol.llh.lat,
        .longitude = gpsSol.llh.lon,
        .altitude = gpsSol.llh.altCm,
        .speed = 0,               // leg default: autopilot cruise
        .duration = 0,
        .type = WAYPOINT_TYPE_FLYBY,
        .pattern = WAYPOINT_PATTERN_NONE,
    };
}

// --- saving -----------------------------------------------------------------

static void writePlan(dispatchEntry_t *self);
static dispatchEntry_t writePlanEntry = { writePlan, 0, NULL, false };

static void writePlan(dispatchEntry_t *self)
{
    UNUSED(self);
    // Never stall the loop with a flash write in flight; a re-arm before the
    // delayed write leaves the plan owed for the next disarm. Unsaved CLI
    // edits are not written behind the pilot's back either.
    if (ARMING_FLAG(ARMED) || !planUnsaved || isConfigDirty()) {
        return;
    }
    writeEEPROM();
    planUnsaved = false;
}

static void requestSave(void)
{
    if (planUnsaved && !recording && !ARMING_FLAG(ARMED)) {
        dispatchEnable();
        dispatchAdd(&writePlanEntry, FP_CAPTURE_SAVE_DELAY_US);
    }
}

// --- AUTO recording ---------------------------------------------------------

// Local flat-earth offset of b from a in centimetres (east, north). Routes
// are a few hundred metres across, well inside the flat-earth error budget.
static void offsetCm(const waypoint_t *a, const waypoint_t *b, float *eastCm, float *northCm)
{
    const float cosLat = cos_approx(DEGREES_TO_RADIANS(a->latitude * 1e-7f));
    *northCm = (float)(b->latitude - a->latitude) * FP_CM_PER_E7_DEG;
    *eastCm = (float)(b->longitude - a->longitude) * FP_CM_PER_E7_DEG * cosLat;
}

// How far point p sits off the line from a to c, horizontally plus its height
// error against the straight climb from a to c. This is what dropping p from
// the route would cost.
static float deviationCm(const waypoint_t *a, const waypoint_t *p, const waypoint_t *c)
{
    float acE, acN, apE, apN;
    offsetCm(a, c, &acE, &acN);
    offsetCm(a, p, &apE, &apN);
    const float acLenSq = sq(acE) + sq(acN);
    float t = 0.0f;
    if (acLenSq > 1.0f) {
        t = constrainf((apE * acE + apN * acN) / acLenSq, 0.0f, 1.0f);
    }
    const float horiz = sqrtf(sq(apE - t * acE) + sq(apN - t * acN));
    const float altOnLine = a->altitude + t * (float)(c->altitude - a->altitude);
    return horiz + FP_REC_THIN_ALT_WEIGHT * fabsf((float)p->altitude - altOnLine);
}

// Make room in a full plan: drop the interior point that matters least. The
// first point (route start) and the last (the reference for the next sample)
// always stay.
static void thinPlan(flightPlanConfig_t *plan)
{
    uint8_t dropIndex = 1;
    float dropCost = INFINITY;
    for (uint8_t i = 1; i + 1 < plan->waypointCount; i++) {
        const float cost = deviationCm(&plan->waypoints[i - 1], &plan->waypoints[i], &plan->waypoints[i + 1]);
        if (cost < dropCost) {
            dropCost = cost;
            dropIndex = i;
        }
    }
    memmove(&plan->waypoints[dropIndex], &plan->waypoints[dropIndex + 1],
            (plan->waypointCount - dropIndex - 1) * sizeof(waypoint_t));
    plan->waypointCount--;
}

static void recordPoint(timeUs_t currentTimeUs, flightPlanConfig_t *plan, const waypoint_t *wp)
{
    if (plan->waypointCount >= MAX_WAYPOINTS) {
        thinPlan(plan);
    }
    plan->waypoints[plan->waypointCount++] = *wp;
    planUnsaved = true;
    setMessage(currentTimeUs, "REC WP%d", plan->waypointCount);
}

// True when the craft has moved far enough from the last recorded point, in
// distance, height or direction, to need a new one.
static bool needsPoint(const flightPlanConfig_t *plan, const waypoint_t *here)
{
    const flightPlanCaptureConfig_t *cfg = flightPlanCaptureConfig();
    const waypoint_t *last = &plan->waypoints[plan->waypointCount - 1];

    float e, n;
    offsetCm(last, here, &e, &n);
    const float distCm = sqrtf(sq(e) + sq(n));

    if (distCm >= cfg->spacingM * 100.0f) {
        return true;
    }
    if (abs(here->altitude - last->altitude) >= cfg->altStepM * 100) {
        return true;
    }
    if (plan->waypointCount >= 2 && distCm >= FP_REC_TURN_MIN_CM) {
        float legE, legN;
        offsetCm(&plan->waypoints[plan->waypointCount - 2], last, &legE, &legN);
        const float legCm = sqrtf(sq(legE) + sq(legN));
        if (legCm >= FP_REC_TURN_MIN_CM) {
            const float cosTurn = (legE * e + legN * n) / (legCm * distCm);
            if (cosTurn < cos_approx(DEGREES_TO_RADIANS(cfg->turnDeg))) {
                return true;
            }
        }
    }
    return false;
}

static void finishRecording(timeUs_t currentTimeUs, flightPlanConfig_t *plan, bool addEndPoint)
{
    recording = false;
    if (addEndPoint && STATE(GPS_FIX) && plan->waypointCount > 0) {
        const waypoint_t here = currentPositionWaypoint();
        const waypoint_t *last = &plan->waypoints[plan->waypointCount - 1];
        float e, n;
        offsetCm(last, &here, &e, &n);
        if (sqrtf(sq(e) + sq(n)) >= FP_REC_END_MIN_CM || abs(here.altitude - last->altitude) >= FP_REC_END_MIN_ALT_CM) {
            recordPoint(currentTimeUs, plan, &here);
        }
    }
    setMessage(currentTimeUs, "ROUTE %d WP", plan->waypointCount);
    requestSave();
}

static void autoUpdate(timeUs_t currentTimeUs, bool switchActive)
{
    flightPlanConfig_t *plan = flightPlanConfigMutable();

    if (!switchActive) {
        if (recording) {
            finishRecording(currentTimeUs, plan, true);
        }
        pressConsumed = false;
        return;
    }

    if (recording) {
        if (!editsAllowed()) {
            // AUTOPILOT took the plan: the route is what was recorded so far
            finishRecording(currentTimeUs, plan, false);
        } else if (STATE(GPS_FIX)) {
            const waypoint_t here = currentPositionWaypoint();
            if (needsPoint(plan, &here)) {
                recordPoint(currentTimeUs, plan, &here);
            }
        }
        return;
    }

    // Switch on, not recording: start once per switch-on, as soon as a fix is
    // available. The old route is only replaced when a new one can begin.
    if (!pressConsumed) {
        if (!editsAllowed()) {
            pressConsumed = true;   // refused, not merely waiting for a fix
        } else if (STATE(GPS_FIX)) {
            pressConsumed = true;
            plan->waypointCount = 0;
            recording = true;
            const waypoint_t here = currentPositionWaypoint();
            recordPoint(currentTimeUs, plan, &here);
        }
    }
}

// Disarm ends a recording even if the switch was left on: the pilot has
// landed, and the save must happen before the battery comes out. The switch
// has to be cycled before it starts another route.
void flightPlanCaptureOnDisarm(void)
{
    if (recording) {
        finishRecording(micros(), flightPlanConfigMutable(), true);
    } else {
        requestSave();
    }
}

// --- TAP gesture --------------------------------------------------------------

static void tapUpdate(timeUs_t currentTimeUs, bool switchActive)
{
    flightPlanConfig_t *plan = flightPlanConfigMutable();

    if (switchActive && !switchWas) {
        // press: sample the position now. Capacity is deliberately not checked
        // here - a hold-to-delete must work on a full plan - only at commit.
        deleteFired = false;
        pendingValid = false;
        pressAccepted = editsAllowed();
        pressTimeUs = currentTimeUs;
        if (pressAccepted && STATE(GPS_FIX)) {
            pending = currentPositionWaypoint();
            pendingValid = true;
        }
    } else if (switchActive && switchWas && !deleteFired
               && cmpTimeUs(currentTimeUs, pressTimeUs) >= (timeDelta_t)FP_CAPTURE_HOLD_DELETE_US) {
        // held: delete the last waypoint, once per press. The release that
        // follows must not also drop the sampled position.
        deleteFired = true;
        pendingValid = false;
        if (pressAccepted && editsAllowed() && plan->waypointCount > 0) {
            plan->waypointCount--;
            planUnsaved = true;
            setMessage(currentTimeUs, "WP%d DELETED", plan->waypointCount + 1);
            requestSave();
        }
    } else if (!switchActive && switchWas && !deleteFired && pressAccepted) {
        // release before the hold threshold: commit the sampled waypoint
        if (pendingValid && editsAllowed()) {
            if (plan->waypointCount < MAX_WAYPOINTS) {
                plan->waypoints[plan->waypointCount++] = pending;
                planUnsaved = true;
                setMessage(currentTimeUs, "WP%d SET", plan->waypointCount);
                requestSave();
            } else {
                setMessage(currentTimeUs, "WP FULL", 0);
            }
        }
        pendingValid = false;
    }
}

void flightPlanCaptureUpdate(timeUs_t currentTimeUs, bool switchActive, bool channelsValid)
{
    // signal loss: freeze the gesture rather than read the dropout as a
    // release. folding validity into switchActive would make a hold that was
    // heading for a delete look like a release and commit the pending waypoint
    // - the exact ghost-edit this guard exists to stop. the press position and
    // the hold clock are both held, so the gesture just resumes when the link
    // is back (and rxfail driving the box high mid-loss still can't touch it).
    // In AUTO the same freeze keeps a dropout from ending the recording.
    if (!channelsValid) {
        return;
    }

    if (flightPlanCaptureConfig()->mode == WP_CAPTURE_MODE_AUTO) {
        autoUpdate(currentTimeUs, switchActive);
    } else {
        tapUpdate(currentTimeUs, switchActive);
    }
    switchWas = switchActive;
}

const char *flightPlanCaptureOsdMessage(void)
{
    if (!msgValid) {
        return NULL;
    }
    if (cmpTimeUs(micros(), msgExpireUs) >= 0) {
        msgValid = false;
        return NULL;
    }
    return msgText;
}

#ifdef UNIT_TEST
void flightPlanCaptureResetForTest(void)
{
    switchWas = false;
    pressTimeUs = 0;
    deleteFired = false;
    pendingValid = false;
    pressAccepted = false;
    recording = false;
    pressConsumed = false;
    planUnsaved = false;
    msgValid = false;
    msgExpireUs = 0;
}
#endif

#endif // ENABLE_FLIGHT_PLAN && !USE_WING
