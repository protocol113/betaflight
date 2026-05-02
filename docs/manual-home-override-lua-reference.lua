--[[
  Manual Home Override - radio-side reference (M9)

  Reference implementation for EdgeTX/OpenTX. Lives here as a design artifact;
  the production deployment lives in a separate radio-side scripts repo.

  Wire format for MSP2_SET_EXTERNAL_HOME (0x300F), 23 bytes little-endian:
    bytes  0- 3  int32   lat        (deg * 1e7)
    bytes  4- 7  int32   lon        (deg * 1e7)
    bytes  8-11  int32   altCm      (cm AMSL)
    bytes 12-13  uint16  donor_pdop (PDOP * 10)
    byte   14    uint8   donor_sats (>= 10)
    bytes 15-18  uint32  capture_ts (unix epoch seconds; 24h freshness check)
    bytes 19-20  uint16  coord_id   (any 16-bit hash; pilot eyeballs last 4 hex)
    bytes 21-22  uint16  crc16      (CRC16-CCITT, init 0, over bytes 0-20)

  FC accepts only when:
    - CRC matches
    - lat/lon in WGS84 range and not (0,0)
    - donor_sats >= 10, donor_pdop <= 20 (PDOP 2.0)
    - capture_ts within 24h of FC RTC (skipped if FC RTC unset)
    - FC disarmed
    - gps_rescue_allow_external_home is on
    - state is NO_HOME or PROVISIONAL (post-VALIDATED rewrites are rejected)
]]

local MSP2_GET_HOME            = 0x300E
local MSP2_SET_EXTERNAL_HOME   = 0x300F
local PAYLOAD_LEN              = 23

-- CRC16-CCITT (poly 0x1021, init 0x0000), bytewise, matches common/crc.c.
local function crc16Ccitt(buf, n)
  local crc = 0
  for i = 1, n do
    crc = bit32.bxor(crc, bit32.lshift(buf[i], 8))
    for _ = 1, 8 do
      if bit32.band(crc, 0x8000) ~= 0 then
        crc = bit32.band(bit32.bxor(bit32.lshift(crc, 1), 0x1021), 0xFFFF)
      else
        crc = bit32.band(bit32.lshift(crc, 1), 0xFFFF)
      end
    end
  end
  return crc
end

local function pushU8(buf, v)  table.insert(buf, bit32.band(v, 0xFF)) end
local function pushU16(buf, v) pushU8(buf, v); pushU8(buf, bit32.rshift(v, 8)) end
local function pushU32(buf, v)
  pushU8(buf, v); pushU8(buf, bit32.rshift(v,  8))
  pushU8(buf, bit32.rshift(v, 16)); pushU8(buf, bit32.rshift(v, 24))
end

-- Build the 23-byte payload from a capture record.
-- capture = { lat, lon, altCm, pdop_x10, sats, captureTs, coordId }
local function buildPayload(capture)
  local buf = {}
  pushU32(buf, capture.lat)
  pushU32(buf, capture.lon)
  pushU32(buf, capture.altCm)
  pushU16(buf, capture.pdop_x10)
  pushU8(buf,  capture.sats)
  pushU32(buf, capture.captureTs)
  pushU16(buf, capture.coordId)
  pushU16(buf, crc16Ccitt(buf, #buf))
  assert(#buf == PAYLOAD_LEN, "payload length")
  return buf
end

-- 16-bit hash so the pilot can eyeball-confirm the same coord on FC OSD as
-- on the radio screen. Two coords that differ by even a few metres collide
-- only ~1/65536 of the time.
local function coordIdFor(lat, lon)
  local h = 0xCBF29CE4    -- 32-bit FNV-ish; we only keep the low 16 bits
  local function mix(x)
    h = bit32.band(bit32.bxor(h, x), 0xFFFFFFFF)
    h = bit32.band(h * 0x01000193, 0xFFFFFFFF)
  end
  mix(bit32.band(lat,                0xFFFF))
  mix(bit32.band(bit32.rshift(lat, 16), 0xFFFF))
  mix(bit32.band(lon,                0xFFFF))
  mix(bit32.band(bit32.rshift(lon, 16), 0xFFFF))
  return bit32.band(h, 0xFFFF)
end

-- Local helpers; replace with the radio's MSP transport API.
local function mspSend(opcode, payload) error("bind to OpenTX/EdgeTX MSP API") end
local function mspRecv(opcode) error("bind to OpenTX/EdgeTX MSP API") end
local function rtcSeconds() error("bind to radio RTC") end

-- Public: capture FC's own GPS now (donor flow). Pilot hovers donor over the
-- intended takeoff point; this reads MSP_RAW_GPS for fix info, then stores
-- the result in the radio's persistent storage as a saved slot.
local function captureFromDonor()
  local raw = mspRecv(0x6A)  -- MSP_RAW_GPS = 106
  if not raw or raw.fix == 0 then return nil, "no_fix" end
  if raw.numSat < 10 then return nil, "low_sats" end
  if raw.pdop > 20 then return nil, "bad_pdop" end
  return {
    lat        = raw.lat,
    lon        = raw.lon,
    altCm      = raw.altCm,
    pdop_x10   = raw.pdop,        -- already PDOP * 10 from FC encoding
    sats       = raw.numSat,
    captureTs  = rtcSeconds(),
    coordId    = coordIdFor(raw.lat, raw.lon),
  }
end

-- Public: send a saved capture to the receiving FC and verify echo via
-- MSP2_GET_HOME. Returns true on confirmed write; false on any failure.
local function pushCapture(capture)
  if rtcSeconds() - capture.captureTs > 24 * 3600 then
    return false, "capture_stale"
  end
  mspSend(MSP2_SET_EXTERNAL_HOME, buildPayload(capture))
  -- Pause briefly for FC ack, then read back state to confirm.
  local home = mspRecv(MSP2_GET_HOME)
  if not home then return false, "no_response" end
  if home.coordId ~= capture.coordId then return false, "coord_id_mismatch" end
  if home.state ~= 1 and home.state ~= 2 then  -- PROVISIONAL or VALIDATED
    return false, "wrong_state"
  end
  return true
end

-- Public: typed coord-ID confirmation gate before reloading a saved slot.
-- Pilot reads the four hex digits off the radio screen and types them back;
-- this prevents a one-button reload of an old / stale capture.
local function confirmAndReload(savedSlot, typedHex)
  if not savedSlot then return false, "no_slot" end
  local expected = string.format("%04X", savedSlot.coordId)
  if string.upper(typedHex) ~= expected then
    return false, "typed_id_mismatch"
  end
  return pushCapture(savedSlot)
end

return {
  captureFromDonor   = captureFromDonor,
  pushCapture        = pushCapture,
  confirmAndReload   = confirmAndReload,
  coordIdFor         = coordIdFor,
  buildPayload       = buildPayload,    -- exported for tests
  crc16Ccitt         = crc16Ccitt,      -- exported for tests
  PAYLOAD_LEN        = PAYLOAD_LEN,
  MSP2_GET_HOME      = MSP2_GET_HOME,
  MSP2_SET_EXTERNAL_HOME = MSP2_SET_EXTERNAL_HOME,
}
