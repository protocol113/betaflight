/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

struct sbuf_s;

typedef enum {
    MANUAL_HOME_STATE_NO_HOME = 0,
    MANUAL_HOME_STATE_PROVISIONAL,
    MANUAL_HOME_STATE_VALIDATED,
    MANUAL_HOME_STATE_REJECTED,
    MANUAL_HOME_STATE_NORMAL,
} manualHomeState_e;

extern manualHomeState_e gpsManualHomeState;
extern uint16_t gpsManualHomeCoordId;

void mspWriteHomeState(struct sbuf_s *dst);
