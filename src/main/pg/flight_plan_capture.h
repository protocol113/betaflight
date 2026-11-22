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

#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    WP_CAPTURE_MODE_TAP = 0,    // tap appends one waypoint, hold deletes the last
    WP_CAPTURE_MODE_AUTO,       // switch on records a route, switch off ends it
    WP_CAPTURE_MODE_COUNT
} wpCaptureMode_e;

// Kept in its own parameter group rather than in flightPlanConfig_t so the
// stored route layout stays identical to upstream's and this fork topic can be
// dropped without touching saved plans.
typedef struct flightPlanCaptureConfig_s {
    uint8_t mode;           // wpCaptureMode_e
    uint8_t spacingM;       // AUTO: horizontal distance between recorded points, metres
    uint8_t altStepM;       // AUTO: height change that forces a point, metres
    uint8_t turnDeg;        // AUTO: change of direction that forces a point, degrees
    uint8_t minAltM;        // AUTO: nothing is recorded below this height above takeoff, metres (0 = no floor)
} flightPlanCaptureConfig_t;

PG_DECLARE(flightPlanCaptureConfig_t, flightPlanCaptureConfig);
