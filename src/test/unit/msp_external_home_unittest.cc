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

extern "C" {
    #include "platform.h"
    #include "common/streambuf.h"
    #include "io/gps.h"
    #include "io/gps_home.h"

    // gps.c provides this in the firmware build; define it here so the unit
    // test can exercise mspWriteHomeState() without linking the full GPS stack.
    gpsLocation_t GPS_home_llh = {};
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

constexpr size_t kPayloadBytes = 15;  // 4 (lat) + 4 (lon) + 4 (alt) + 1 (state) + 2 (coord_id)

void serializeHome(uint8_t *buf, size_t bufLen)
{
    sbuf_t sbuf;
    sbuf.ptr = buf;
    sbuf.end = buf + bufLen;
    mspWriteHomeState(&sbuf);
    EXPECT_EQ(sbuf.ptr, buf + kPayloadBytes);
}

int32_t readI32LE(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

uint16_t readU16LE(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

}  // namespace

TEST(MspExternalHomeUnitTest, GetHomeDefaultIsNoHome)
{
    GPS_home_llh.lat = 0;
    GPS_home_llh.lon = 0;
    GPS_home_llh.altCm = 0;
    gpsManualHomeState = MANUAL_HOME_STATE_NO_HOME;
    gpsManualHomeCoordId = 0;

    uint8_t buf[kPayloadBytes] = {};
    serializeHome(buf, sizeof(buf));

    EXPECT_EQ(readI32LE(&buf[0]), 0);
    EXPECT_EQ(readI32LE(&buf[4]), 0);
    EXPECT_EQ(readI32LE(&buf[8]), 0);
    EXPECT_EQ(buf[12], MANUAL_HOME_STATE_NO_HOME);
    EXPECT_EQ(readU16LE(&buf[13]), 0);
}

TEST(MspExternalHomeUnitTest, GetHomeReportsProvisionalStateAndCoordId)
{
    // 40.7128456, -74.0059821, alt 1234cm
    GPS_home_llh.lat = 407128456;
    GPS_home_llh.lon = -740059821;
    GPS_home_llh.altCm = 1234;
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomeCoordId = 0xABCD;

    uint8_t buf[kPayloadBytes] = {};
    serializeHome(buf, sizeof(buf));

    EXPECT_EQ(readI32LE(&buf[0]), 407128456);
    EXPECT_EQ(readI32LE(&buf[4]), -740059821);
    EXPECT_EQ(readI32LE(&buf[8]), 1234);
    EXPECT_EQ(buf[12], MANUAL_HOME_STATE_PROVISIONAL);
    EXPECT_EQ(readU16LE(&buf[13]), 0xABCD);
}

TEST(MspExternalHomeUnitTest, GetHomeRoundTripsAllStateValues)
{
    GPS_home_llh.lat = 1;
    GPS_home_llh.lon = 2;
    GPS_home_llh.altCm = 3;
    gpsManualHomeCoordId = 0;

    const manualHomeState_e states[] = {
        MANUAL_HOME_STATE_NO_HOME,
        MANUAL_HOME_STATE_PROVISIONAL,
        MANUAL_HOME_STATE_VALIDATED,
        MANUAL_HOME_STATE_REJECTED,
        MANUAL_HOME_STATE_NORMAL,
    };

    for (manualHomeState_e s : states) {
        gpsManualHomeState = s;
        uint8_t buf[kPayloadBytes] = {};
        serializeHome(buf, sizeof(buf));
        EXPECT_EQ(buf[12], s);
    }
}
