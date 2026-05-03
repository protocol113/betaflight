--[[
  Manual Home Override - radio TOOLS script (M9 deployable)

  Drop this on your radio's SD card under /SCRIPTS/TOOLS/manualhome.lua and
  it appears in the Tools menu (long-press SYS on EdgeTX). Tested on EdgeTX
  2.10 with ELRS/CRSF; should work on OpenTX 2.3+ with the same transport.
  For non-CRSF radios (FrSky SBUS, Multiprotocol, etc.) replace the two
  transport functions below.

  Pilot UX:
    Page 1 (idle):                         [ENTER]   capture from FC GPS
    Page 2 (capture preview):  shows lat/lon/sats/pdop/coord_id
                                           [ENTER]   push to FC
                                           [EXIT]    discard
    Page 3 (pushed):           shows coord_id and FC-reported state
                                           [ENTER]   refresh GET_HOME
                                           [EXIT]    return to idle

  The coord_id printed on screen is what pilot must eyeball-confirm against
  the OSD warning HOME EXT XXXX. Identical hashes -> same point.
]]

------------------------------------------------------------------------
-- Wire constants (must match src/main/io/gps_home.h on the FC side)
------------------------------------------------------------------------
local MSP_RAW_GPS              = 106     -- existing MSP1 opcode
local MSP2_GET_HOME            = 0x300E
local MSP2_SET_EXTERNAL_HOME   = 0x300F
local PAYLOAD_LEN              = 23

local STATE_NAMES = {
  [0] = "NO_HOME", [1] = "PROVIS",  [2] = "VALID",
  [3] = "REJECT",  [4] = "NORMAL",
}

------------------------------------------------------------------------
-- CRC16-CCITT (poly 0x1021, init 0x0000) -- byte-exact with common/crc.c
------------------------------------------------------------------------
local function crc16(buf, n)
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

local function pushU8 (b, v) b[#b+1] = bit32.band(v, 0xFF) end
local function pushU16(b, v) pushU8(b, v); pushU8(b, bit32.rshift(v, 8)) end
local function pushU32(b, v)
  pushU8(b, v); pushU8(b, bit32.rshift(v, 8))
  pushU8(b, bit32.rshift(v, 16)); pushU8(b, bit32.rshift(v, 24))
end

local function readU16(b, off) return b[off] + b[off+1] * 256 end
local function readI32(b, off)
  local v = b[off] + b[off+1] * 256 + b[off+2] * 65536 + b[off+3] * 16777216
  if v >= 2147483648 then v = v - 4294967296 end
  return v
end

------------------------------------------------------------------------
-- coord_id: 16-bit FNV-style hash of (lat, lon). Two captures of the
-- same point produce the same coord_id; ~1/65536 collision rate.
------------------------------------------------------------------------
local function coordIdFor(lat, lon)
  local h = 0xCBF29CE4
  local function mix(x)
    h = bit32.band(bit32.bxor(h, x), 0xFFFFFFFF)
    -- 32-bit multiply that survives Lua's 53-bit double precision
    h = bit32.band((h * 16777619) % 0x100000000, 0xFFFFFFFF)
  end
  mix(bit32.band(lat, 0xFFFF))
  mix(bit32.band(bit32.rshift(lat, 16), 0xFFFF))
  mix(bit32.band(lon, 0xFFFF))
  mix(bit32.band(bit32.rshift(lon, 16), 0xFFFF))
  return bit32.band(h, 0xFFFF)
end

------------------------------------------------------------------------
-- MSP V2 transport over CRSF subtype 0x2D (ELRS / Crossfire / TBS).
-- For other radio links, replace these two functions.
------------------------------------------------------------------------
local CRSF_FRAMETYPE_MSP_REQ  = 0x7A
local CRSF_FRAMETYPE_MSP_RESP = 0x7B
local CRSF_ADDR_FC            = 0xC8
local CRSF_ADDR_RADIO         = 0xEA

-- MSP-over-telemetry status byte for a single-fragment V2 frame.
-- See src/main/telemetry/msp_shared.c on the FC side:
--   bits 0-3 = sequence (0 for first/only)
--   bit 4    = MSP_STATUS_START_MASK (must be set on first fragment)
--   bits 5-6 = MSP version (2 = MSP V2)
--   bit 7    = error
local MSP_STATUS_V2_START = 0x50

-- Send an MSP V2 request (or set, if payload is non-empty).
-- function_id is the MSP V2 opcode (e.g. 0x300F).
-- The CRSF transport on the FC side computes its own outer frame CRC,
-- and the MSP-over-telemetry envelope does NOT include an inner V2 CRC8
-- (unlike serial MSP V2). So the data we push is exactly:
--   {dest, origin, status, flags, fn_lo, fn_hi, sz_lo, sz_hi, ...payload}
local function mspSend(function_id, payload)
  local frame = {
    CRSF_ADDR_FC,
    CRSF_ADDR_RADIO,
    MSP_STATUS_V2_START,
    0,                              -- V2 flags
    bit32.band(function_id, 0xFF),
    bit32.rshift(function_id, 8),
    bit32.band(#payload, 0xFF),
    bit32.rshift(#payload, 8),
  }
  for i = 1, #payload do frame[#frame+1] = payload[i] end
  crossfireTelemetryPush(CRSF_FRAMETYPE_MSP_REQ, frame)
end

-- Drain whatever's queued and return the latest decoded MSP V2 response
-- as { fn=opcode, data={byte, byte, ...} } or nil if none.
-- crossfireTelemetryPop returns the frame body after the type byte:
--   frame[1] = dest, [2] = origin, [3] = status, [4] = V2 flags,
--   [5..6]   = fn_lo / fn_hi
--   [7..8]   = size_lo / size_hi
--   [9..]    = payload
local function mspRecv()
  local cmd, frame = crossfireTelemetryPop()
  if not cmd or cmd ~= CRSF_FRAMETYPE_MSP_RESP then return nil end
  if not frame or #frame < 8 then return nil end
  -- Ignore continuation frames; we don't reassemble multi-fragment responses
  -- (every opcode this script uses fits comfortably in one CRSF frame).
  if bit32.band(frame[3], 0x10) == 0 then return nil end
  local fn   = frame[5] + frame[6] * 256
  local size = frame[7] + frame[8] * 256
  local data = {}
  for i = 1, size do data[i] = frame[8 + i] end
  return { fn = fn, data = data }
end

------------------------------------------------------------------------
-- Capture / push helpers
------------------------------------------------------------------------
local function buildSetHomePayload(c)
  local b = {}
  pushU32(b, c.lat)
  pushU32(b, c.lon)
  pushU32(b, c.altCm)
  pushU16(b, c.pdop_x10)
  pushU8 (b, c.sats)
  pushU32(b, c.captureTs)
  pushU16(b, c.coordId)
  pushU16(b, crc16(b, #b))
  return b
end

local function parseRawGpsResponse(d)
  if #d < 18 then return nil, "short_resp:" .. #d end
  -- MSP_RAW_GPS layout (1-indexed in Lua):
  --   [1]    fix (0/1)
  --   [2]    numSat
  --   [3..6] lat (int32, deg * 1e7)
  --   [7..10] lon (int32, deg * 1e7)
  --   [11..12] alt (uint16, metres)
  --   [13..14] groundSpeed
  --   [15..16] groundCourse
  --   [17..18] PDOP (uint16, PDOP * 100 -- documented in io/gps.h)
  if d[1] == 0 then return nil, "no_fix" end
  local sats = d[2]
  if sats < 10 then return nil, "low_sats:" .. sats end
  -- Convert PDOP * 100 (MSP_RAW_GPS encoding) to PDOP * 10
  -- (MSP2_SET_EXTERNAL_HOME encoding) by dividing. The FC validator
  -- caps donor_pdop at 20 (PDOP 2.0), so reject here too.
  local pdop_x100 = readU16(d, 17)
  local pdop_x10  = math.floor(pdop_x100 / 10)
  if pdop_x10 > 20 then
    return nil, string.format("bad_pdop %.2f", pdop_x100 / 100)
  end
  return {
    lat       = readI32(d, 3),
    lon       = readI32(d, 7),
    altCm     = readU16(d, 11) * 100, -- alt in m -> cm
    sats      = sats,
    pdop_x10  = pdop_x10,
    captureTs = getRtcTime(),
    coordId   = coordIdFor(readI32(d, 3), readI32(d, 7)),
  }
end

local function parseGetHomeResponse(d)
  if #d < 15 then return nil end
  return {
    lat     = readI32(d, 1),
    lon     = readI32(d, 5),
    altCm   = readI32(d, 9),
    state   = d[13],
    coordId = readU16(d, 14),
  }
end

------------------------------------------------------------------------
-- Script state machine
------------------------------------------------------------------------
local PAGE_IDLE     = 1
local PAGE_PREVIEW  = 2
local PAGE_PUSHED   = 3

local page          = PAGE_IDLE
local capture       = nil    -- captured donor record (table)
local pushedAt      = 0
local lastHome      = nil    -- last MSP2_GET_HOME response
local statusLine    = "press [ENTER] to capture"

local function requestCapture()
  mspSend(MSP_RAW_GPS, {})
  statusLine = "requesting RAW_GPS..."
end

local function pushCapture()
  if not capture then return end
  if getRtcTime() - capture.captureTs > 24 * 3600 then
    statusLine = "capture stale (>24h)"
    return
  end
  mspSend(MSP2_SET_EXTERNAL_HOME, buildSetHomePayload(capture))
  pushedAt = getTime()
  page = PAGE_PUSHED
  statusLine = "pushed; awaiting echo"
  -- chain a GET to confirm
  mspSend(MSP2_GET_HOME, {})
end

local function refreshHome()
  mspSend(MSP2_GET_HOME, {})
  statusLine = "refreshing..."
end

local function background()
  -- drain everything available in this tick
  while true do
    local r = mspRecv()
    if not r then break end
    if r.fn == MSP_RAW_GPS and page == PAGE_IDLE then
      local rec, err = parseRawGpsResponse(r.data)
      if rec then
        capture = rec
        page = PAGE_PREVIEW
        statusLine = "captured; [ENTER]=push  [EXIT]=discard"
      else
        statusLine = "capture failed: " .. (err or "?")
      end
    elseif r.fn == MSP2_GET_HOME then
      lastHome = parseGetHomeResponse(r.data)
      if lastHome then
        statusLine = string.format("FC state=%s id=%04X",
                                   STATE_NAMES[lastHome.state] or "?",
                                   lastHome.coordId)
      end
    end
  end
end

------------------------------------------------------------------------
-- UI
------------------------------------------------------------------------
local function drawHeader(title)
  lcd.clear()
  lcd.drawText(2, 0, "Manual Home Override")
  lcd.drawText(2, 10, title, INVERS)
  lcd.drawLine(0, 20, LCD_W, 20, SOLID, FORCE)
end

local function drawIdle()
  drawHeader("[1] Idle")
  lcd.drawText(2, 24, "ENTER: capture from FC GPS")
  if lastHome then
    lcd.drawText(2, 38, string.format("Last GET: %s id=%04X",
                                       STATE_NAMES[lastHome.state] or "?",
                                       lastHome.coordId))
  else
    lcd.drawText(2, 38, "no FC data yet")
  end
end

local function drawPreview()
  drawHeader("[2] Capture preview")
  if not capture then return end
  lcd.drawText(2, 24, string.format("lat %d", capture.lat))
  lcd.drawText(2, 32, string.format("lon %d", capture.lon))
  lcd.drawText(2, 40, string.format("sats=%d pdop=%.1f",
                                     capture.sats, capture.pdop_x10 / 10))
  lcd.drawText(2, 48, string.format("ID %04X (eyeball this)",
                                     capture.coordId), INVERS)
end

local function drawPushed()
  drawHeader("[3] Pushed")
  if capture then
    lcd.drawText(2, 24, string.format("Sent ID %04X", capture.coordId))
  end
  if lastHome then
    lcd.drawText(2, 34, string.format("FC state: %s",
                                       STATE_NAMES[lastHome.state] or "?"))
    lcd.drawText(2, 42, string.format("FC id: %04X", lastHome.coordId))
    if capture and lastHome.coordId == capture.coordId then
      lcd.drawText(2, 52, "MATCH", INVERS)
    end
  else
    lcd.drawText(2, 34, "waiting for echo")
  end
end

local function init() end

local function run(event)
  background()

  if page == PAGE_IDLE then
    drawIdle()
    if event == EVT_VIRTUAL_ENTER then requestCapture() end
    if event == EVT_VIRTUAL_EXIT  then return 2 end
  elseif page == PAGE_PREVIEW then
    drawPreview()
    if event == EVT_VIRTUAL_ENTER then pushCapture() end
    if event == EVT_VIRTUAL_EXIT  then capture = nil; page = PAGE_IDLE end
  elseif page == PAGE_PUSHED then
    drawPushed()
    if event == EVT_VIRTUAL_ENTER then refreshHome() end
    if event == EVT_VIRTUAL_EXIT  then page = PAGE_IDLE end
    -- Auto-poll every ~500ms while on this page so the state machine
    -- shows the FC promoting PROVIS -> VALID without pilot input.
    if (getTime() - pushedAt) > 50 then
      pushedAt = getTime()
      mspSend(MSP2_GET_HOME, {})
    end
  end

  lcd.drawText(2, LCD_H - 8, statusLine)
  return 0
end

return { init = init, run = run, background = background }
