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

#include <stdbool.h>
#include <stdint.h>

struct sbuf_s;

typedef enum {
    MANUAL_HOME_STATE_NO_HOME = 0,
    MANUAL_HOME_STATE_PROVISIONAL,
    MANUAL_HOME_STATE_VALIDATED,
    MANUAL_HOME_STATE_REJECTED,
    MANUAL_HOME_STATE_NORMAL,
} manualHomeState_e;

// Wire format for MSP2_SET_EXTERNAL_HOME. CRC16-CCITT covers the first 21 bytes.
#define EXTERNAL_HOME_PAYLOAD_BYTES 23

// Quality gates the donor must meet for the FC to accept its capture.
#define EXTERNAL_HOME_MIN_DONOR_SATS    10
#define EXTERNAL_HOME_MAX_DONOR_PDOP    20      // PDOP 2.0 (×10)
#define EXTERNAL_HOME_MAX_AGE_SECONDS   (24U * 3600U)

typedef enum {
    EXTERNAL_HOME_OK = 0,
    EXTERNAL_HOME_ERR_TRUNCATED,
    EXTERNAL_HOME_ERR_CRC,
    EXTERNAL_HOME_ERR_RANGE,
    EXTERNAL_HOME_ERR_SATS,
    EXTERNAL_HOME_ERR_PDOP,
    EXTERNAL_HOME_ERR_STALE,
    EXTERNAL_HOME_ERR_ARMED,
    EXTERNAL_HOME_ERR_DISABLED,
    EXTERNAL_HOME_ERR_LOCKED,
} externalHomeResult_e;

extern manualHomeState_e gpsManualHomeState;
extern uint16_t gpsManualHomeCoordId;

void mspWriteHomeState(struct sbuf_s *dst);

// Validates and applies an MSP2_SET_EXTERNAL_HOME payload. On EXTERNAL_HOME_OK
// the function has updated GPS_home_llh, set STATE(GPS_FIX_HOME), advanced
// gpsManualHomeState to PROVISIONAL, and stored the coord_id. On any other
// return value, no state changed.
externalHomeResult_e processExternalHomeMessage(struct sbuf_s *src,
                                                uint32_t nowEpochSeconds,
                                                bool fcArmed,
                                                bool featureEnabled);
