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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_GPS

#include "common/crc.h"
#include "common/streambuf.h"

#include "fc/runtime_config.h"

#include "io/gps.h"
#include "io/gps_home.h"

#define EXTERNAL_HOME_LAT_LIMIT    900000000   // 90 degrees * 1e7
#define EXTERNAL_HOME_LON_LIMIT    1800000000  // 180 degrees * 1e7
#define EXTERNAL_HOME_PAYLOAD_CRC_OFFSET (EXTERNAL_HOME_PAYLOAD_BYTES - 2)

manualHomeState_e gpsManualHomeState = MANUAL_HOME_STATE_NO_HOME;
uint16_t gpsManualHomeCoordId = 0;

void mspWriteHomeState(sbuf_t *dst)
{
    sbufWriteU32(dst, GPS_home_llh.lat);
    sbufWriteU32(dst, GPS_home_llh.lon);
    sbufWriteU32(dst, GPS_home_llh.altCm);
    sbufWriteU8(dst, (uint8_t)gpsManualHomeState);
    sbufWriteU16(dst, gpsManualHomeCoordId);
}

externalHomeResult_e processExternalHomeMessage(sbuf_t *src,
                                                uint32_t nowEpochSeconds,
                                                bool fcArmed,
                                                bool featureEnabled)
{
    if (sbufBytesRemaining(src) < EXTERNAL_HOME_PAYLOAD_BYTES) {
        return EXTERNAL_HOME_ERR_TRUNCATED;
    }

    const uint8_t *payload = src->ptr;
    const uint16_t receivedCrc = (uint16_t)payload[EXTERNAL_HOME_PAYLOAD_CRC_OFFSET]
                               | ((uint16_t)payload[EXTERNAL_HOME_PAYLOAD_CRC_OFFSET + 1] << 8);
    const uint16_t computedCrc = crc16_ccitt_update(0, payload, EXTERNAL_HOME_PAYLOAD_CRC_OFFSET);
    if (receivedCrc != computedCrc) {
        return EXTERNAL_HOME_ERR_CRC;
    }

    const int32_t  lat              = (int32_t)sbufReadU32(src);
    const int32_t  lon              = (int32_t)sbufReadU32(src);
    const int32_t  altCm            = (int32_t)sbufReadU32(src);
    const uint16_t donorPdop        = sbufReadU16(src);
    const uint8_t  donorSatCount    = sbufReadU8(src);
    const uint32_t captureTimestamp = sbufReadU32(src);
    const uint16_t coordId          = sbufReadU16(src);
    sbufReadU16(src); // CRC, already verified

    if ((lat == 0 && lon == 0)
        || lat >  EXTERNAL_HOME_LAT_LIMIT || lat < -EXTERNAL_HOME_LAT_LIMIT
        || lon >  EXTERNAL_HOME_LON_LIMIT || lon < -EXTERNAL_HOME_LON_LIMIT) {
        return EXTERNAL_HOME_ERR_RANGE;
    }
    if (donorSatCount < EXTERNAL_HOME_MIN_DONOR_SATS) {
        return EXTERNAL_HOME_ERR_SATS;
    }
    if (donorPdop > EXTERNAL_HOME_MAX_DONOR_PDOP) {
        return EXTERNAL_HOME_ERR_PDOP;
    }
    // capture_timestamp must not be older than the configured window. A
    // capture from the future (clock skew) is treated as fresh. nowEpochSeconds
    // == 0 means the FC has no RTC sync; the staleness check is skipped because
    // any comparison against an unset clock is meaningless.
    if (nowEpochSeconds != 0
        && captureTimestamp <= nowEpochSeconds
        && (nowEpochSeconds - captureTimestamp) > EXTERNAL_HOME_MAX_AGE_SECONDS) {
        return EXTERNAL_HOME_ERR_STALE;
    }
    if (fcArmed) {
        return EXTERNAL_HOME_ERR_ARMED;
    }
    if (!featureEnabled) {
        return EXTERNAL_HOME_ERR_DISABLED;
    }
    // Only NO_HOME and PROVISIONAL accept external writes. Once the FC has its
    // own fix or has decided the manual home is unsafe, the external coord
    // can't override that for the rest of the powered session.
    if (gpsManualHomeState != MANUAL_HOME_STATE_NO_HOME
        && gpsManualHomeState != MANUAL_HOME_STATE_PROVISIONAL) {
        return EXTERNAL_HOME_ERR_LOCKED;
    }

    GPS_home_llh.lat = lat;
    GPS_home_llh.lon = lon;
    GPS_home_llh.altCm = altCm;
    gpsManualHomeCoordId = coordId;
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    ENABLE_STATE(GPS_FIX_HOME);

    return EXTERNAL_HOME_OK;
}

#endif // USE_GPS
