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

#include <stdint.h>

#include "platform.h"

#ifdef USE_GPS

#include "common/streambuf.h"

#include "io/gps.h"
#include "io/gps_home.h"

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

#endif // USE_GPS
