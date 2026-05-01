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
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "common/crc.h"
    #include "common/streambuf.h"
    #include "fc/runtime_config.h"
    #include "io/gps.h"
    #include "io/gps_home.h"

    // gps.c provides this in the firmware build; define it here so the unit
    // test can exercise mspWriteHomeState() without linking the full GPS stack.
    gpsLocation_t GPS_home_llh = {};

    // runtime_config.c references beeperConfirmationBeeps when toggling
    // flight modes. The validator never enables/disables a flight mode, so
    // this stub is sufficient for the linker.
    void beeperConfirmationBeeps(uint8_t) {}
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

constexpr size_t kGetHomePayloadBytes = 15;  // 4 (lat) + 4 (lon) + 4 (alt) + 1 (state) + 2 (coord_id)

void serializeHome(uint8_t *buf, size_t bufLen)
{
    sbuf_t sbuf;
    sbuf.ptr = buf;
    sbuf.end = buf + bufLen;
    mspWriteHomeState(&sbuf);
    EXPECT_EQ(sbuf.ptr, buf + kGetHomePayloadBytes);
}

int32_t readI32LE(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

uint16_t readU16LE(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

void writeU8(uint8_t *p, size_t &off, uint8_t v) { p[off++] = v; }
void writeU16LE(uint8_t *p, size_t &off, uint16_t v)
{
    p[off++] = (uint8_t)(v & 0xff);
    p[off++] = (uint8_t)((v >> 8) & 0xff);
}
void writeU32LE(uint8_t *p, size_t &off, uint32_t v)
{
    p[off++] = (uint8_t)(v & 0xff);
    p[off++] = (uint8_t)((v >> 8) & 0xff);
    p[off++] = (uint8_t)((v >> 16) & 0xff);
    p[off++] = (uint8_t)((v >> 24) & 0xff);
}

struct ExternalHomePayload {
    int32_t  lat = 407128456;
    int32_t  lon = -740059821;
    int32_t  altCm = 1234;
    uint16_t donorPdop = 15;        // 1.5
    uint8_t  donorSatCount = 12;
    uint32_t captureTimestamp = 1700000000;
    uint16_t coordId = 0xABCD;
};

// Build a 23-byte payload with a valid CRC16-CCITT trailer.
size_t buildPayload(uint8_t *buf, const ExternalHomePayload &p, bool corruptCrc = false)
{
    size_t off = 0;
    writeU32LE(buf, off, (uint32_t)p.lat);
    writeU32LE(buf, off, (uint32_t)p.lon);
    writeU32LE(buf, off, (uint32_t)p.altCm);
    writeU16LE(buf, off, p.donorPdop);
    writeU8(buf, off, p.donorSatCount);
    writeU32LE(buf, off, p.captureTimestamp);
    writeU16LE(buf, off, p.coordId);
    const uint16_t crc = crc16_ccitt_update(0, buf, off);
    writeU16LE(buf, off, corruptCrc ? (uint16_t)(crc ^ 0xffff) : crc);
    EXPECT_EQ(off, (size_t)EXTERNAL_HOME_PAYLOAD_BYTES);
    return off;
}

// Wrap a buffer in an sbuf_t and run the validator.
externalHomeResult_e parse(const uint8_t *buf, size_t len, uint32_t now, bool armed, bool enabled)
{
    sbuf_t sbuf;
    sbuf.ptr = (uint8_t *)buf;
    sbuf.end = (uint8_t *)buf + len;
    return processExternalHomeMessage(&sbuf, now, armed, enabled);
}

void resetHomeStateForTest()
{
    GPS_home_llh.lat = 0;
    GPS_home_llh.lon = 0;
    GPS_home_llh.altCm = 0;
    gpsManualHomeState = MANUAL_HOME_STATE_NO_HOME;
    gpsManualHomeCoordId = 0;
    DISABLE_STATE(GPS_FIX_HOME);
}

constexpr uint32_t kNowSec = 1700001000; // 1000s after capture default

}  // namespace

// ---------- M1: MSP_GET_HOME serialization ----------

TEST(MspExternalHomeUnitTest, GetHomeDefaultIsNoHome)
{
    resetHomeStateForTest();
    uint8_t buf[kGetHomePayloadBytes] = {};
    serializeHome(buf, sizeof(buf));

    EXPECT_EQ(readI32LE(&buf[0]), 0);
    EXPECT_EQ(readI32LE(&buf[4]), 0);
    EXPECT_EQ(readI32LE(&buf[8]), 0);
    EXPECT_EQ(buf[12], MANUAL_HOME_STATE_NO_HOME);
    EXPECT_EQ(readU16LE(&buf[13]), 0);
}

TEST(MspExternalHomeUnitTest, GetHomeReportsProvisionalStateAndCoordId)
{
    GPS_home_llh.lat = 407128456;
    GPS_home_llh.lon = -740059821;
    GPS_home_llh.altCm = 1234;
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomeCoordId = 0xABCD;

    uint8_t buf[kGetHomePayloadBytes] = {};
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
        uint8_t buf[kGetHomePayloadBytes] = {};
        serializeHome(buf, sizeof(buf));
        EXPECT_EQ(buf[12], s);
    }
}

// ---------- M3: MSP2_SET_EXTERNAL_HOME validation ----------

TEST(MspSetExternalHomeUnitTest, HappyPathTransitionsToProvisional)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_OK);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    EXPECT_EQ(GPS_home_llh.lat, p.lat);
    EXPECT_EQ(GPS_home_llh.lon, p.lon);
    EXPECT_EQ(GPS_home_llh.altCm, p.altCm);
    EXPECT_EQ(gpsManualHomeCoordId, p.coordId);
    EXPECT_TRUE(STATE(GPS_FIX_HOME));
}

TEST(MspSetExternalHomeUnitTest, RejectsTruncatedPayload)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES - 1] = {};
    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_TRUNCATED);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_NO_HOME);
    EXPECT_FALSE(STATE(GPS_FIX_HOME));
}

TEST(MspSetExternalHomeUnitTest, RejectsBadCrc)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p, /*corruptCrc=*/true);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_CRC);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_NO_HOME);
}

TEST(MspSetExternalHomeUnitTest, RejectsZeroZeroCoord)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.lat = 0;
    p.lon = 0;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_RANGE);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_NO_HOME);
}

TEST(MspSetExternalHomeUnitTest, RejectsOutOfRangeLat)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.lat = 1000000000;  // 100 deg, beyond 90 deg cap
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_RANGE);
}

TEST(MspSetExternalHomeUnitTest, RejectsOutOfRangeLon)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.lon = -1900000000;  // -190 deg
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_RANGE);
}

TEST(MspSetExternalHomeUnitTest, RejectsLowSatCount)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.donorSatCount = 9;  // min is 10
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_SATS);
}

TEST(MspSetExternalHomeUnitTest, RejectsHighPdop)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.donorPdop = 21;  // max is 20 (PDOP 2.0)
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_PDOP);
}

TEST(MspSetExternalHomeUnitTest, RejectsStaleCapture)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    // captureTimestamp default 1700000000; advance now by >24h
    const uint32_t now = p.captureTimestamp + EXTERNAL_HOME_MAX_AGE_SECONDS + 1;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), now, false, true), EXTERNAL_HOME_ERR_STALE);
}

TEST(MspSetExternalHomeUnitTest, AcceptsCaptureExactlyAtAgeLimit)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    const uint32_t now = p.captureTimestamp + EXTERNAL_HOME_MAX_AGE_SECONDS;  // exactly at limit
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), now, false, true), EXTERNAL_HOME_OK);
}

TEST(MspSetExternalHomeUnitTest, SkipsAgeCheckWhenRtcUnset)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.captureTimestamp = 100;  // very old, but...
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), 0 /*RTC unset*/, false, true), EXTERNAL_HOME_OK);
}

TEST(MspSetExternalHomeUnitTest, RejectsWhenArmed)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, /*armed=*/true, true), EXTERNAL_HOME_ERR_ARMED);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_NO_HOME);
}

TEST(MspSetExternalHomeUnitTest, RejectsWhenFeatureDisabled)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, /*enabled=*/false), EXTERNAL_HOME_ERR_DISABLED);
}

TEST(MspSetExternalHomeUnitTest, RejectsWhenStateValidated)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_VALIDATED;
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.coordId = 0xBEEF;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_LOCKED);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);
    EXPECT_NE(gpsManualHomeCoordId, 0xBEEF);  // unchanged
}

TEST(MspSetExternalHomeUnitTest, RejectsWhenStateRejected)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_REJECTED;
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_LOCKED);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_REJECTED);
}

TEST(MspSetExternalHomeUnitTest, RejectsWhenStateNormalHome)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_NORMAL;
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_ERR_LOCKED);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_NORMAL);
}

TEST(MspSetExternalHomeUnitTest, IdempotentReSendInProvisional)
{
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p);

    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_OK);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);

    // Rebuild a fresh sbuf and resend the same payload.
    EXPECT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_OK);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    EXPECT_EQ(gpsManualHomeCoordId, p.coordId);
}

TEST(MspSetExternalHomeUnitTest, NewCoordIdInProvisionalUpdates)
{
    resetHomeStateForTest();
    uint8_t buf1[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p1;
    buildPayload(buf1, p1);
    ASSERT_EQ(parse(buf1, sizeof(buf1), kNowSec, false, true), EXTERNAL_HOME_OK);

    uint8_t buf2[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p2;
    p2.lat = 351234567;     // different field
    p2.lon = -1180000000;
    p2.coordId = 0x1234;
    buildPayload(buf2, p2);

    EXPECT_EQ(parse(buf2, sizeof(buf2), kNowSec, false, true), EXTERNAL_HOME_OK);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    EXPECT_EQ(GPS_home_llh.lat, p2.lat);
    EXPECT_EQ(GPS_home_llh.lon, p2.lon);
    EXPECT_EQ(gpsManualHomeCoordId, p2.coordId);
}

// ---------- M4: First-fix promotion (PROVISIONAL -> VALIDATED / REJECTED) ----------

namespace {

manualHomePromotionInputs_t makeGoodFix(uint32_t distanceCm)
{
    manualHomePromotionInputs_t in = {};
    in.fcHasFix = true;
    in.fcSatCount = 12;
    in.fcPdopX10 = 15;          // PDOP 1.5
    in.distanceToHomeCm = distanceCm;
    in.minSats = 8;
    in.maxHomeDistanceM = 1500;
    in.maxPdopX10 = 25;         // PDOP 2.5
    return in;
}

void prePromote(uint32_t distanceCm, uint8_t ticks)
{
    auto in = makeGoodFix(distanceCm);
    for (uint8_t i = 0; i < ticks; ++i) {
        gpsManualHomePromotionTick(&in);
    }
}

}  // namespace

TEST(GpsManualHomePromotionTest, ThreeCloseFixesPromoteToValidated)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    auto in = makeGoodFix(80000);  // 800m, well under 1500m

    gpsManualHomePromotionTick(&in);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    gpsManualHomePromotionTick(&in);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    gpsManualHomePromotionTick(&in);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);
}

TEST(GpsManualHomePromotionTest, ThreeFarFixesRejectHome)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    auto in = makeGoodFix(500000);  // 5000m, well over 1500m

    gpsManualHomePromotionTick(&in);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    gpsManualHomePromotionTick(&in);
    gpsManualHomePromotionTick(&in);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_REJECTED);
}

TEST(GpsManualHomePromotionTest, ExactlyAtMaxDistanceCounts)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    auto in = makeGoodFix(150000);  // exactly 1500m, treated as close

    gpsManualHomePromotionTick(&in);
    gpsManualHomePromotionTick(&in);
    gpsManualHomePromotionTick(&in);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);
}

TEST(GpsManualHomePromotionTest, BadFixResetsCloseCounter)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    auto good = makeGoodFix(80000);
    gpsManualHomePromotionTick(&good);
    gpsManualHomePromotionTick(&good);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);

    auto bad = good;
    bad.fcSatCount = 4;  // sat dip below minSats
    gpsManualHomePromotionTick(&bad);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);

    // After the reset, two more close fixes shouldn't be enough.
    gpsManualHomePromotionTick(&good);
    gpsManualHomePromotionTick(&good);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);

    // Third good fix completes the new run.
    gpsManualHomePromotionTick(&good);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);
}

TEST(GpsManualHomePromotionTest, HighPdopBlocksPromotion)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    auto in = makeGoodFix(80000);
    in.fcPdopX10 = 30;  // PDOP 3.0, above 2.5 cap

    for (int i = 0; i < 5; ++i) {
        gpsManualHomePromotionTick(&in);
    }
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
}

TEST(GpsManualHomePromotionTest, NoFixResetsCounters)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    auto good = makeGoodFix(80000);
    gpsManualHomePromotionTick(&good);
    gpsManualHomePromotionTick(&good);

    auto noFix = good;
    noFix.fcHasFix = false;
    gpsManualHomePromotionTick(&noFix);

    // Resume good fixes; need a fresh run of three.
    gpsManualHomePromotionTick(&good);
    gpsManualHomePromotionTick(&good);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    gpsManualHomePromotionTick(&good);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);
}

TEST(GpsManualHomePromotionTest, FlippingVerdictResetsRunningCount)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    auto close = makeGoodFix(80000);
    auto far = makeGoodFix(500000);

    gpsManualHomePromotionTick(&close);
    gpsManualHomePromotionTick(&close);
    gpsManualHomePromotionTick(&far);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);

    // far counter is at 1; need two more fars to reject.
    gpsManualHomePromotionTick(&far);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    gpsManualHomePromotionTick(&far);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_REJECTED);
}

TEST(GpsManualHomePromotionTest, ValidatedIsSticky)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();
    prePromote(80000, 3);
    ASSERT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);

    auto far = makeGoodFix(500000);
    for (int i = 0; i < 10; ++i) {
        gpsManualHomePromotionTick(&far);
    }
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);
}

TEST(GpsManualHomePromotionTest, RejectedIsSticky)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();
    prePromote(500000, 3);
    ASSERT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_REJECTED);

    auto close = makeGoodFix(80000);
    for (int i = 0; i < 10; ++i) {
        gpsManualHomePromotionTick(&close);
    }
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_REJECTED);
}

TEST(GpsManualHomePromotionTest, NotPromotedFromNoHomeState)
{
    resetHomeStateForTest();
    // state already MANUAL_HOME_STATE_NO_HOME
    auto close = makeGoodFix(80000);
    for (int i = 0; i < 10; ++i) {
        gpsManualHomePromotionTick(&close);
    }
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_NO_HOME);
}

// ---------- M5: rescue activation gate ----------

namespace {

rescueGateInputs_t gateFor(manualHomeState_e s, bool magOk, uint32_t distCm)
{
    rescueGateInputs_t in = {};
    in.state = s;
    in.magHealthy = magOk;
    in.distanceFlownCm = distCm;
    in.yawConvergeDistCm = GPS_RESCUE_YAW_CONVERGE_DIST_CM;
    return in;
}

}  // namespace

TEST(GpsRescueGateTest, NoHomeAllowsRescueDispatch)
{
    // NO_HOME means no manual home was loaded; the existing FC rescue path
    // gates on STATE(GPS_FIX_HOME) downstream. The new gate should not
    // override that decision either way.
    auto in = gateFor(MANUAL_HOME_STATE_NO_HOME, false, 0);
    EXPECT_TRUE(gpsRescueShouldFire(&in));
}

TEST(GpsRescueGateTest, NormalHomeAllowsRescueDispatch)
{
    // FC's own GPS produced the home -- existing rescue, no new gating.
    auto in = gateFor(MANUAL_HOME_STATE_NORMAL, false, 0);
    EXPECT_TRUE(gpsRescueShouldFire(&in));
}

TEST(GpsRescueGateTest, ProvisionalAlwaysDrops)
{
    // Rule #6: rescue is inert until the FC has its own fix and validates
    // the manual home. Drop on failsafe instead of flying somewhere unsafe.
    auto in1 = gateFor(MANUAL_HOME_STATE_PROVISIONAL, true, 100000);
    EXPECT_FALSE(gpsRescueShouldFire(&in1));

    auto in2 = gateFor(MANUAL_HOME_STATE_PROVISIONAL, false, 0);
    EXPECT_FALSE(gpsRescueShouldFire(&in2));
}

TEST(GpsRescueGateTest, RejectedAlwaysDrops)
{
    auto in1 = gateFor(MANUAL_HOME_STATE_REJECTED, true, 100000);
    EXPECT_FALSE(gpsRescueShouldFire(&in1));

    auto in2 = gateFor(MANUAL_HOME_STATE_REJECTED, false, 0);
    EXPECT_FALSE(gpsRescueShouldFire(&in2));
}

TEST(GpsRescueGateTest, ValidatedFiresWithMag)
{
    // Magnetometer healthy -> yaw is trustable immediately, no distance
    // requirement.
    auto in = gateFor(MANUAL_HOME_STATE_VALIDATED, true, 0);
    EXPECT_TRUE(gpsRescueShouldFire(&in));
}

TEST(GpsRescueGateTest, ValidatedDropsBelowYawConvergenceWithoutMag)
{
    // No mag, drone hasn't moved enough for COG-derived yaw to settle.
    auto in = gateFor(MANUAL_HOME_STATE_VALIDATED, false, 1000);  // 10m
    EXPECT_FALSE(gpsRescueShouldFire(&in));
}

TEST(GpsRescueGateTest, ValidatedFiresAtYawConvergenceWithoutMag)
{
    // Exactly at threshold -> COG yaw considered converged.
    auto in = gateFor(MANUAL_HOME_STATE_VALIDATED, false, GPS_RESCUE_YAW_CONVERGE_DIST_CM);
    EXPECT_TRUE(gpsRescueShouldFire(&in));
}

TEST(GpsRescueGateTest, ValidatedFiresAboveYawConvergenceWithoutMag)
{
    auto in = gateFor(MANUAL_HOME_STATE_VALIDATED, false, GPS_RESCUE_YAW_CONVERGE_DIST_CM + 1);
    EXPECT_TRUE(gpsRescueShouldFire(&in));
}

TEST(GpsRescueGateTest, AcceptingExternalHomeReturnsToProvisionalGate)
{
    // After a fresh external write, gate must drop until promotion completes.
    resetHomeStateForTest();
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    buildPayload(buf, p);
    ASSERT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_OK);

    auto in = gateFor(gpsManualHomeState, true, 100000);  // magnetometer healthy and lots of distance
    EXPECT_FALSE(gpsRescueShouldFire(&in)) << "Fresh PROVISIONAL must drop on failsafe even with good mag";
}

// ---------- M4: extra promotion regression ----------

TEST(GpsManualHomePromotionTest, AcceptingNewExternalHomeResetsPromotionCounters)
{
    resetHomeStateForTest();
    gpsManualHomeState = MANUAL_HOME_STATE_PROVISIONAL;
    gpsManualHomePromotionReset();

    // Build up two close votes
    auto close = makeGoodFix(80000);
    gpsManualHomePromotionTick(&close);
    gpsManualHomePromotionTick(&close);

    // A new external home arrives -- counters must reset, otherwise the next
    // tick after the new home would promote it without three independent fixes.
    uint8_t buf[EXTERNAL_HOME_PAYLOAD_BYTES];
    ExternalHomePayload p;
    p.coordId = 0x9999;
    buildPayload(buf, p);
    ASSERT_EQ(parse(buf, sizeof(buf), kNowSec, false, true), EXTERNAL_HOME_OK);
    ASSERT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);

    gpsManualHomePromotionTick(&close);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    gpsManualHomePromotionTick(&close);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_PROVISIONAL);
    gpsManualHomePromotionTick(&close);
    EXPECT_EQ(gpsManualHomeState, MANUAL_HOME_STATE_VALIDATED);
}
