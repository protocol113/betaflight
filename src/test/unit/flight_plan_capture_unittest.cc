/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "common/time.h"

    #include "fc/runtime_config.h"

    #include "flight/flight_plan_capture.h"

    #include "io/gps.h"

    #include "pg/flight_plan.h"
    #include "pg/flight_plan_capture.h"
    #include "pg/pg.h"

    #include "fc/dispatch.h"

    uint8_t stateFlags;
    uint8_t armingFlags;
    gpsSolutionData_t gpsSol;
}

#include <cmath>

#include "unittest_macros.h"
#include "gtest/gtest.h"

// --- Stubs ---

namespace {
timeUs_t g_stubMicros;
bool g_stubNavActive;
bool g_stubConfigDirty;
int g_eepromWrites;
dispatchEntry_t *g_pendingDispatch;
} // namespace

extern "C" {

timeUs_t micros(void)
{
    return g_stubMicros;
}

bool flightPlanNavIsActive(void)
{
    return g_stubNavActive;
}

void dispatchEnable(void) {}

void dispatchAdd(dispatchEntry_t *entry, int delayUs)
{
    UNUSED(delayUs);
    g_pendingDispatch = entry;
}

bool isConfigDirty(void)
{
    return g_stubConfigDirty;
}

void writeEEPROM(void)
{
    g_eepromWrites++;
}

} // extern "C"

// 1e-7 degree of latitude in centimetres (WGS84 mean, as used by the FC)
static const double CM_PER_E7_DEG = 1.113195;
static const int32_t START_LAT = 500000000;   // 50 deg N
static const int32_t START_LON = -1200000000;
static const int32_t START_ALT_CM = 3000;

class FlightPlanCaptureTest : public ::testing::Test {
protected:
    void SetUp() override {
        flightPlanCaptureResetForTest();
        memset(&gpsSol, 0, sizeof(gpsSol));
        flightPlanConfigMutable()->waypointCount = 0;
        // shipped defaults: TAP, 10 m spacing, 3 m height step, 25 deg turn
        *flightPlanCaptureConfigMutable() = { WP_CAPTURE_MODE_TAP, 10, 3, 25 };
        stateFlags = GPS_FIX;
        armingFlags = 0;
        g_stubNavActive = false;
        g_stubConfigDirty = false;
        g_eepromWrites = 0;
        g_pendingDispatch = NULL;
        g_stubMicros = 1000000;
        setPosition(START_LAT, START_LON, START_ALT_CM);
    }

    // move relative to the current position, in metres
    void move(double northM, double eastM, double upM = 0.0) {
        const double cosLat = cos(gpsSol.llh.lat * 1e-7 * M_PI / 180.0);
        gpsSol.llh.lat += (int32_t)lround(northM * 100.0 / CM_PER_E7_DEG);
        gpsSol.llh.lon += (int32_t)lround(eastM * 100.0 / (CM_PER_E7_DEG * cosLat));
        gpsSol.llh.altCm += (int32_t)lround(upM * 100.0);
    }

    // one control-loop tick 20 ms later with the switch in the given state
    void tick(bool switchActive, bool channelsValid = true) {
        advanceMs(20);
        update(switchActive, channelsValid);
    }

    void setAuto(void) {
        flightPlanCaptureConfigMutable()->mode = WP_CAPTURE_MODE_AUTO;
    }

    // true when the stored waypoint i is within toleranceM of the position
    // northM/eastM metres from the start
    bool waypointNear(uint8_t i, double northM, double eastM, double toleranceM = 0.5) {
        const waypoint_t *wp = &flightPlanConfig()->waypoints[i];
        const double cosLat = cos(START_LAT * 1e-7 * M_PI / 180.0);
        const double n = (wp->latitude - START_LAT) * CM_PER_E7_DEG / 100.0;
        const double e = (wp->longitude - START_LON) * CM_PER_E7_DEG * cosLat / 100.0;
        return std::hypot(n - northM, e - eastM) <= toleranceM;
    }

    void runPendingSave(void) {
        ASSERT_TRUE(g_pendingDispatch != NULL);
        dispatchEntry_t *entry = g_pendingDispatch;
        g_pendingDispatch = NULL;
        entry->dispatch(entry);
    }

    void setPosition(int32_t lat, int32_t lon, int32_t altCm) {
        gpsSol.llh.lat = lat;
        gpsSol.llh.lon = lon;
        gpsSol.llh.altCm = altCm;
    }

    void advanceMs(uint32_t ms) {
        g_stubMicros += ms * 1000;
    }

    void update(bool switchActive, bool channelsValid = true) {
        flightPlanCaptureUpdate(g_stubMicros, switchActive, channelsValid);
    }

    // press then release with the switch held for holdMs
    void tap(uint32_t holdMs = 100) {
        update(true);
        advanceMs(holdMs);
        update(false);
    }

    void fillPlan(uint8_t count) {
        flightPlanConfig_t *plan = flightPlanConfigMutable();
        plan->waypointCount = count;
        for (uint8_t i = 0; i < count; i++) {
            plan->waypoints[i] = { 100 + i, 200 + i, 1000, 0, 0, WAYPOINT_TYPE_FLYBY, WAYPOINT_PATTERN_NONE };
        }
    }
};

TEST_F(FlightPlanCaptureTest, TapAppendsWaypointSampledAtThePress)
{
    update(true);                                  // press samples here
    setPosition(500001000, -1200001000, 4500);     // craft drifts before release
    advanceMs(200);
    update(false);                                 // commit

    const flightPlanConfig_t *plan = flightPlanConfig();
    ASSERT_EQ(1, plan->waypointCount);
    EXPECT_EQ(500000000, plan->waypoints[0].latitude);
    EXPECT_EQ(-1200000000, plan->waypoints[0].longitude);
    EXPECT_EQ(3000, plan->waypoints[0].altitude);
    EXPECT_EQ(WAYPOINT_TYPE_FLYBY, plan->waypoints[0].type);
    EXPECT_EQ(0, plan->waypoints[0].speed);       // leg default cruise
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    EXPECT_STREQ("WP1 SET", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, HoldDeletesTheLastWaypointAndTheReleaseDropsNothing)
{
    fillPlan(3);

    update(true);
    advanceMs(1600);                               // past the 1.5 s threshold
    update(true);                                  // delete fires while held
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_STREQ("WP3 DELETED", flightPlanCaptureOsdMessage());

    advanceMs(400);
    update(false);                                 // release must not also drop
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, HoldDeleteStillWorksOnAFullPlan)
{
    fillPlan(MAX_WAYPOINTS);

    update(true);
    advanceMs(1600);
    update(true);
    EXPECT_EQ(MAX_WAYPOINTS - 1, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, TapOnAFullPlanAddsNothingAndSaysSo)
{
    fillPlan(MAX_WAYPOINTS);

    tap();
    EXPECT_EQ(MAX_WAYPOINTS, flightPlanConfig()->waypointCount);
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    EXPECT_STREQ("WP FULL", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, NoFixMeansNoWaypoint)
{
    stateFlags = 0;

    tap();
    EXPECT_EQ(0, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);
}

TEST_F(FlightPlanCaptureTest, SwitchIsIgnoredWhileTheMissionFlies)
{
    fillPlan(2);
    g_stubNavActive = true;

    tap();                                         // no append
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);

    update(true);
    advanceMs(1600);
    update(true);                                  // no delete either
    update(false);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);
}

TEST_F(FlightPlanCaptureTest, SignalLossMidHoldFreezesTheGestureNotCommitsIt)
{
    fillPlan(2);

    // press starts a hold that's heading for a delete...
    update(true);
    advanceMs(800);
    // ...but the link drops before the 1.5 s threshold. that must not read as a
    // release and commit the pending waypoint - freeze instead
    update(true, false);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);

    // link back, still held, cross the threshold: the delete lands as intended
    advanceMs(800);
    update(true);
    EXPECT_EQ(1, flightPlanConfig()->waypointCount);
    EXPECT_STREQ("WP2 DELETED", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, MissionEngagingBetweenPressAndReleaseCancelsTheDrop)
{
    update(true);
    advanceMs(200);
    g_stubNavActive = true;                        // executor takes the plan mid-gesture
    update(false);
    EXPECT_EQ(0, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, ConsecutiveTapsNumberTheWaypoints)
{
    tap();
    advanceMs(500);
    tap();
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    EXPECT_STREQ("WP2 SET", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, OsdMessageExpires)
{
    tap();
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    advanceMs(2400);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    advanceMs(200);                                // past the 2.5 s cue lifetime
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);
}

// --- AUTO mode: the switch records a route while it is on ---

TEST_F(FlightPlanCaptureTest, AutoSwitchOnReplacesTheRouteWithTheStartPoint)
{
    setAuto();
    fillPlan(5);                                   // an older route

    tick(true);

    const flightPlanConfig_t *plan = flightPlanConfig();
    ASSERT_EQ(1, plan->waypointCount);
    EXPECT_EQ(START_LAT, plan->waypoints[0].latitude);
    EXPECT_EQ(START_LON, plan->waypoints[0].longitude);
    EXPECT_EQ(START_ALT_CM, plan->waypoints[0].altitude);
    EXPECT_EQ(WAYPOINT_TYPE_FLYBY, plan->waypoints[0].type);
    EXPECT_EQ(0, plan->waypoints[0].speed);
    EXPECT_STREQ("REC WP1", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, AutoAddsAPointEachTimeTheSpacingIsCovered)
{
    setAuto();
    tick(true);

    move(9.0, 0.0);                                // short of the 10 m spacing
    tick(true);
    EXPECT_EQ(1, flightPlanConfig()->waypointCount);

    move(1.5, 0.0);                                // 10.5 m from the last point
    tick(true);
    ASSERT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(waypointNear(1, 10.5, 0.0));
    EXPECT_STREQ("REC WP2", flightPlanCaptureOsdMessage());

    move(10.5, 0.0);
    tick(true);
    EXPECT_EQ(3, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoAddsAPointOnAHeightChange)
{
    setAuto();
    tick(true);

    move(0.0, 0.0, 2.5);                           // under the 3 m step
    tick(true);
    EXPECT_EQ(1, flightPlanConfig()->waypointCount);

    move(0.0, 0.0, 0.7);                           // 3.2 m above the last point
    tick(true);
    ASSERT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_EQ(START_ALT_CM + 320, flightPlanConfig()->waypoints[1].altitude);
}

TEST_F(FlightPlanCaptureTest, AutoAddsAPointWhenTheCraftTurns)
{
    setAuto();
    tick(true);
    move(10.5, 0.0);                               // north leg
    tick(true);
    ASSERT_EQ(2, flightPlanConfig()->waypointCount);

    move(0.0, 4.0);                                // 4 m east: a 90 deg turn
    tick(true);
    ASSERT_EQ(3, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(waypointNear(2, 10.5, 4.0));
}

TEST_F(FlightPlanCaptureTest, AutoIgnoresTurnsOverTinyDistances)
{
    // GPS jitter sideways must not read as a turn
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);

    move(0.0, 1.5);                                // under the 3 m turn minimum
    tick(true);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoKeepsGentleCurvesBelowTheTurnThreshold)
{
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);

    move(5.0, 1.5);                                // ~17 deg off the north leg
    tick(true);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoSwitchOffEndsTheRouteAtTheCurrentPosition)
{
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);
    move(4.0, 0.0);
    tick(false);

    ASSERT_EQ(3, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(waypointNear(2, 14.5, 0.0));
    EXPECT_STREQ("ROUTE 3 WP", flightPlanCaptureOsdMessage());

    // the switch staying off changes nothing
    move(30.0, 0.0);
    tick(false);
    EXPECT_EQ(3, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoSwitchOffRightOnTheLastPointAddsNoDuplicate)
{
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);
    move(0.5, 0.0);
    tick(false);

    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoFlippingOnAgainStartsAFreshRoute)
{
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);
    tick(false);
    ASSERT_EQ(2, flightPlanConfig()->waypointCount);

    move(50.0, 0.0);
    tick(true);
    ASSERT_EQ(1, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(waypointNear(0, 61.0, 0.0));
}

TEST_F(FlightPlanCaptureTest, AutoWaitsForAFixBeforeTouchingTheRoute)
{
    setAuto();
    fillPlan(4);
    stateFlags = 0;

    tick(true);                                    // no fix: keep the old route
    EXPECT_EQ(4, flightPlanConfig()->waypointCount);

    stateFlags = GPS_FIX;                          // fix arrives, switch still on
    tick(true);
    ASSERT_EQ(1, flightPlanConfig()->waypointCount);
    EXPECT_EQ(START_LAT, flightPlanConfig()->waypoints[0].latitude);
}

TEST_F(FlightPlanCaptureTest, AutoSkipsSamplingWhileTheFixIsLost)
{
    setAuto();
    tick(true);

    stateFlags = 0;
    move(20.0, 0.0);                               // stale or bad position
    tick(true);
    EXPECT_EQ(1, flightPlanConfig()->waypointCount);

    stateFlags = GPS_FIX;
    tick(true);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoSignalLossIsNotASwitchOff)
{
    setAuto();
    tick(true);
    move(5.0, 0.0);

    // rxfail can drive the aux channel low during a dropout: no final point
    tick(false, false);
    EXPECT_EQ(1, flightPlanConfig()->waypointCount);

    // link back with the switch still on: recording carries on
    move(6.0, 0.0);
    tick(true);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(waypointNear(1, 11.0, 0.0));
}

TEST_F(FlightPlanCaptureTest, AutoDoesNotStartWhileTheMissionFlies)
{
    setAuto();
    fillPlan(3);
    g_stubNavActive = true;

    tick(true);
    move(20.0, 0.0);
    tick(true);
    tick(false);
    EXPECT_EQ(3, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoStopsRecordingWhenTheMissionEngages)
{
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);
    ASSERT_EQ(2, flightPlanConfig()->waypointCount);

    g_stubNavActive = true;                        // pilot flips AUTOPILOT on
    move(20.0, 0.0);
    tick(true);
    tick(false);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);

    // and the switch does not resume recording when the mission ends
    g_stubNavActive = false;
    tick(false);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, AutoFullRouteDropsTheLeastSignificantPoint)
{
    setAuto();
    tick(true);
    // 10 m north, then a right-angle corner, then a long east leg
    move(10.5, 0.0);
    tick(true);
    move(10.5, 0.0);
    tick(true);                                    // wp2: the corner at 21 m north
    for (int i = 0; flightPlanConfig()->waypointCount < MAX_WAYPOINTS; i++) {
        move(0.0, 10.5);
        tick(true);
        ASSERT_LT(i, 100);
    }
    ASSERT_EQ(MAX_WAYPOINTS, flightPlanConfig()->waypointCount);

    move(0.0, 10.5);
    tick(true);                                    // one past full

    const flightPlanConfig_t *plan = flightPlanConfig();
    EXPECT_EQ(MAX_WAYPOINTS, plan->waypointCount);
    EXPECT_TRUE(waypointNear(0, 0.0, 0.0));        // start kept
    EXPECT_TRUE(waypointNear(2, 21.0, 0.0) || waypointNear(1, 21.0, 0.0));  // corner kept
    const double lastEast = 10.5 * (MAX_WAYPOINTS - 2);
    EXPECT_TRUE(waypointNear(MAX_WAYPOINTS - 1, 21.0, lastEast));           // newest is last
}

TEST_F(FlightPlanCaptureTest, AutoLongRecordingKeepsTheCornersOfASquare)
{
    flightPlanCaptureConfigMutable()->spacingM = 5;
    setAuto();
    tick(true);
    // a 200 m square walked in 2.5 m steps: ~320 samples, far beyond 30 points
    const double legs[4][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
    for (int leg = 0; leg < 4; leg++) {
        for (int step = 0; step < 80; step++) {
            move(2.5 * legs[leg][0], 2.5 * legs[leg][1]);
            tick(true);
        }
    }
    tick(false);

    const flightPlanConfig_t *plan = flightPlanConfig();
    ASSERT_EQ(MAX_WAYPOINTS, plan->waypointCount);
    const double corners[4][2] = { { 200, 0 }, { 200, 200 }, { 0, 200 }, { 0, 0 } };
    for (int c = 0; c < 4; c++) {
        bool found = false;
        for (uint8_t i = 0; i < plan->waypointCount; i++) {
            found |= waypointNear(i, corners[c][0], corners[c][1], 3.0);
        }
        EXPECT_TRUE(found) << "corner " << c << " lost";
    }
}

TEST_F(FlightPlanCaptureTest, TapModeIsUnaffectedBySpacing)
{
    // default TAP: holding the switch on while moving records nothing extra
    update(true);
    move(30.0, 0.0);
    advanceMs(300);
    update(true);
    update(false);
    EXPECT_EQ(1, flightPlanConfig()->waypointCount);
}

// --- saving the route without a laptop ---

TEST_F(FlightPlanCaptureTest, RouteRecordedWhileDisarmedIsSavedWhenTheSwitchGoesOff)
{
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);
    EXPECT_TRUE(g_pendingDispatch == NULL);        // nothing while recording

    tick(false);
    runPendingSave();
    EXPECT_EQ(1, g_eepromWrites);
}

TEST_F(FlightPlanCaptureTest, RouteRecordedInFlightIsSavedAfterDisarm)
{
    armingFlags = ARMED;
    setAuto();
    tick(true);
    move(10.5, 0.0);
    tick(true);
    tick(false);
    EXPECT_TRUE(g_pendingDispatch == NULL);        // never write flash while armed

    armingFlags = 0;
    flightPlanCaptureOnDisarm();
    runPendingSave();
    EXPECT_EQ(1, g_eepromWrites);

    // nothing new: the next disarm does not write again
    flightPlanCaptureOnDisarm();
    EXPECT_TRUE(g_pendingDispatch == NULL);
}

TEST_F(FlightPlanCaptureTest, SaveIsSkippedWhenThereAreUnsavedCliChanges)
{
    setAuto();
    tick(true);
    tick(false);
    g_stubConfigDirty = true;
    runPendingSave();
    EXPECT_EQ(0, g_eepromWrites);
}

TEST_F(FlightPlanCaptureTest, SaveWaitsIfTheCraftIsReArmedFirst)
{
    armingFlags = ARMED;
    setAuto();
    tick(true);
    tick(false);
    armingFlags = 0;
    flightPlanCaptureOnDisarm();
    armingFlags = ARMED;                           // quick re-arm before the delayed save
    runPendingSave();
    EXPECT_EQ(0, g_eepromWrites);

    armingFlags = 0;
    flightPlanCaptureOnDisarm();                   // still owed: saved this time
    runPendingSave();
    EXPECT_EQ(1, g_eepromWrites);
}

TEST_F(FlightPlanCaptureTest, TapEditsAreAlsoSavedAfterDisarm)
{
    armingFlags = ARMED;
    tap();
    armingFlags = 0;
    flightPlanCaptureOnDisarm();
    runPendingSave();
    EXPECT_EQ(1, g_eepromWrites);
}
