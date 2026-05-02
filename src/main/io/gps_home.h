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

// Transition counters indexed by destination state. Visible to tests for
// asserting that beeper/blackbox events fired the expected number of times.
extern uint16_t gpsManualHomeTransitionCount[5];

void mspWriteHomeState(struct sbuf_s *dst);

// Validates and applies an MSP2_SET_EXTERNAL_HOME payload. On EXTERNAL_HOME_OK
// the function has updated GPS_home_llh, set STATE(GPS_FIX_HOME), advanced
// gpsManualHomeState to PROVISIONAL, and stored the coord_id. On any other
// return value, no state changed.
externalHomeResult_e processExternalHomeMessage(struct sbuf_s *src,
                                                uint32_t nowEpochSeconds,
                                                bool fcArmed,
                                                bool featureEnabled);

// Number of consecutive good fixes that must agree before the FC promotes
// the manual home to VALIDATED or REJECTED.
#define MANUAL_HOME_PROMOTION_FIX_COUNT 3

typedef struct {
    bool     fcHasFix;          // true if the FC currently has a 3D fix
    uint8_t  fcSatCount;        // current sat count from gpsSol.numSat
    uint16_t fcPdopX10;         // current PDOP * 10 (gpsSol.dop.pdop / 10)
    uint32_t distanceToHomeCm;  // GPS_distanceToHomeCm (FC -> manual home)
    uint8_t  minSats;           // gpsRescueConfig()->minSats
    uint16_t maxHomeDistanceM;  // gpsRescueConfig()->maxHomeDistanceM
    uint8_t  maxPdopX10;        // gpsRescueConfig()->maxPdop
} manualHomePromotionInputs_t;

// Drive the PROVISIONAL -> VALIDATED / REJECTED state machine using the
// FC's own GPS data. Sticky once VALIDATED or REJECTED is reached. Counters
// reset on a bad fix. Caller invokes this once per onGpsNewData() tick.
void gpsManualHomePromotionTick(const manualHomePromotionInputs_t *in);

// Reset the internal consecutive-fix counters. Call this when the manual
// home itself is replaced (e.g. PROVISIONAL re-write with a new coord_id)
// so the existing fix history doesn't carry over to a different home.
void gpsManualHomePromotionReset(void);

// Distance (cm) the FC must have flown for COG-derived yaw to be trusted
// when no magnetometer is available. Below this threshold the rescue gate
// rejects activation in MANUAL_HOME_STATE_VALIDATED to avoid wrong-direction
// flight in the first few seconds.
#define GPS_RESCUE_YAW_CONVERGE_DIST_CM 3000U  // 30 metres

typedef struct {
    manualHomeState_e state;
    bool     magHealthy;        // SENSOR_MAG present, useMag set, !magForceDisable
    uint32_t distanceFlownCm;   // GPS_distanceFlownInCm
    uint32_t yawConvergeDistCm; // GPS_RESCUE_YAW_CONVERGE_DIST_CM (parameter for testability)
} rescueGateInputs_t;

// Returns true iff GPS Rescue should activate on failsafe given the current
// manual-home state and yaw situation. The caller (failsafe.c) falls through
// to a drop when this returns false.
//   NO_HOME / NORMAL  -> allow (existing rescue path; STATE(GPS_FIX_HOME) gates
//                       the no-home case downstream).
//   PROVISIONAL       -> drop (rule #6: rescue is inert until promoted).
//   REJECTED          -> drop (manual home was disagreed with by FC's own fix).
//   VALIDATED         -> allow iff yaw is trustable: magHealthy, OR the FC has
//                       flown far enough for COG-derived yaw to converge.
bool gpsRescueShouldFire(const rescueGateInputs_t *in);


