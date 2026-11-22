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

#include "platform.h"

#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/flight_plan_capture.h"

PG_REGISTER_WITH_RESET_TEMPLATE(flightPlanCaptureConfig_t, flightPlanCaptureConfig, PG_FLIGHT_PLAN_CAPTURE_CONFIG, 0);

PG_RESET_TEMPLATE(flightPlanCaptureConfig_t, flightPlanCaptureConfig,
    .mode = WP_CAPTURE_MODE_TAP,
    .spacingM = 10,
    .altStepM = 3,
    .turnDeg = 25,
    .minAltM = 5,
);

#endif // ENABLE_FLIGHT_PLAN && !USE_WING
