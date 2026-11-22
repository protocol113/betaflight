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

#pragma once

#include <stdbool.h>

#include "common/time.h"

// In-flight waypoint capture on the WP CAPTURE box. Edits apply to the live
// PG plan, so the CLI waypoint command sees them and `save` persists them.
//
// wp_capture_mode = TAP: a tap appends the craft's position (sampled at the
// press) as a FLYBY waypoint, a 1.5 s hold deletes the last waypoint.
//
// wp_capture_mode = AUTO: switching on replaces the plan with the current
// position and records a route while the switch stays on - a point every
// wp_capture_spacing_m, on a wp_capture_alt_m height change, or on a turn of
// wp_capture_turn_deg. Switching off adds the end point. A full plan drops the
// point whose removal changes the route shape least.
// Nothing is recorded below wp_capture_min_alt_m above takeoff, a switch
// already on at power-up is ignored until cycled, and disarming ends the
// recording without adding a point.
//
// In both modes an edited plan is written to flash once the craft is
// disarmed (at switch-off if already disarmed, otherwise after disarm).
void flightPlanCaptureUpdate(timeUs_t currentTimeUs, bool switchActive, bool channelsValid);

// Call on every disarm: schedules the flash write for a plan edited in flight.
void flightPlanCaptureOnDisarm(void);

// Transient OSD cue ("WP3 SET", "WP2 DELETED", "WP FULL", "REC WP4",
// "ROUTE 12 WP"), or NULL when none is pending. The cue self-expires.
const char *flightPlanCaptureOsdMessage(void);

#ifdef UNIT_TEST
void flightPlanCaptureResetForTest(void);
#endif
