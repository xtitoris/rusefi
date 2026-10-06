-- This script makes a second rusEFI ECU emulate the DEVICE side of the
-- MegaSquirt/Microsquirt "CAN I/O Box" protocol, so the ECU-side driver in
-- firmware/hw_layer/drivers/gpio/can_gpio_msiobox.cpp can be exercised
-- against a live CAN peer instead of real IO-Box hardware.
--
-- Protocol reference: "Microsquirt as I/O Box" manual (James Murray,
-- 2014-12-27), section 6 "Programmers reference to CAN communications",
-- cross-checked against can_gpio_msiobox.cpp. All multi-byte protocol
-- fields are big-endian (MOTOROLA byte order).
--
-- Packet map (CAN id = baseId + offset):
--   ECU -> device:
--     +0  ping, 0 bytes. Device replies with +8 (whoami).
--     +1  config: pwm_mask, pad, tachin_mask, pad, adc_interval_ms, tach_interval_ms, pad, pad
--     +2  PWM1,2 "on"/"off" periods (sent only if either channel is in PWM mode)
--     +3  PWM3,4 "on"/"off" periods (sent only if either channel is in PWM mode)
--     +4  PWM5,6 "on"/"off" periods (sent only if either channel is in PWM mode)
--     +5  PWM7 "on"/"off" period + on/off outputs bitfield (sent always)
--   device -> ECU:
--     +8  whoami: version, pad x3, PWM clock period, tach-in clock period (both 0.01us units)
--     +9  ADC1..ADC4, 10-bit raw (0-5V max input)
--     +10 switch-inputs bitfield, pad, ADC5..ADC7, 10-bit raw (0-5V max input)
--     +11..+14 Tach 1..4: accumulated period (device tach-clock ticks) over
--              N teeth, teeth count, running total-tooth counter;
--              only broadcast while that tach input is enabled (tachin_mask)

setTickRate(300)

local bus = 1
local baseId = 0x240
local isExtId = 0

local PWM_COUNT = 7              -- PWM1..PWM7 / OUT1..OUT7
local DEFAULT_BROADCAST_MS = 20  -- manual's documented default for both ADC and tach broadcast

-- device clock constants we advertise in whoami - matches the real device's
-- documented defaults, in 0.01us units
local PWM_CLOCK_PERIOD = 5000    -- -> 20000 Hz tick rate for PWM on/off counting
local TACHIN_CLOCK_PERIOD = 66   -- -> ~1,515,151 Hz tick rate for tach period counting
local PWM_CLOCK_HZ = 100000000 / PWM_CLOCK_PERIOD

------------------------------------------------------------
-- big-endian byte packing/unpacking helpers (offset is 0-based, like the C structs)
------------------------------------------------------------

function setByte(data, offset, value)
	data[offset + 1] = math.floor(value) & 0xff
end

function getByte(data, offset)
	return data[offset + 1]
end

-- MOTOROLA order, MSB (Most Significant Byte/Big Endian) comes first
function setTwoBytesMsb(data, offset, value)
	value = math.floor(value)
	data[offset + 1] = (value >> 8) & 0xff
	data[offset + 2] = value & 0xff
end

function getTwoBytesMsb(data, offset)
	return (data[offset + 1] << 8) | data[offset + 2]
end

function setFourBytesMsb(data, offset, value)
	value = math.floor(value)
	data[offset + 1] = (value >> 24) & 0xff
	data[offset + 2] = (value >> 16) & 0xff
	data[offset + 3] = (value >> 8) & 0xff
	data[offset + 4] = value & 0xff
end

function clamp01(x)
	if x < 0 then return 0 end
	if x > 1 then return 1 end
	return x
end

------------------------------------------------------------
-- device state, as configured by the ECU
------------------------------------------------------------

local pwmMask = 0                     -- bit per channel: 0 = on/off, 1 = PWM
local tachinMask = 0                  -- bit per tach input: 1 = enabled
local outState = 0                    -- bit per channel: on/off level, used when that channel is not in PWM mode
local pwmOn = {0, 0, 0, 0, 0, 0, 0}    -- per-channel "on" tick count, as last sent by the ECU
local pwmOff = {0, 0, 0, 0, 0, 0, 0}   -- per-channel "off" tick count
local totalTeeth = {0, 0, 0, 0}        -- running per-tach tooth counter, wraps at 16 bits

local adcIntervalMs = DEFAULT_BROADCAST_MS
local tachIntervalMs = DEFAULT_BROADCAST_MS

local adcTimer = Timer.new()
local tachTimer = Timer.new()

-- reserve one Lua PWM channel per IO-box output; plain on/off outputs are
-- emulated by driving a PWM channel at a fixed duty of 0 or 1
for ch = 0, PWM_COUNT - 1 do
	startPwm(ch, 100, 0)
end

-- Re-drive every output pin from the current pwmMask/pwmOn/pwmOff/outState.
-- Called whenever any of those change.
function applyOutputs()
	for ch = 0, PWM_COUNT - 1 do
		if (pwmMask & (1 << ch)) ~= 0 then
			-- PWM mode: on/off tick counts -> frequency + duty
			local total = pwmOn[ch + 1] + pwmOff[ch + 1]
			if total > 0 then
				setPwmFreq(ch, PWM_CLOCK_HZ / total)
				setPwmDuty(ch, clamp01(pwmOn[ch + 1] / total))
			else
				setPwmDuty(ch, 0)
			end
		else
			-- plain on/off mode: drive the level straight from outState
			setPwmDuty(ch, (outState >> ch) & 1)
		end
	end
end

------------------------------------------------------------
-- ECU -> device packet handlers
------------------------------------------------------------

function onPing(bus, id, dlc, data)
	print('MSIO: got ping, replying with whoami')

	-- a fresh ping means the ECU (re)started its handshake - reset to a
	-- known-good default state, same as a freshly powered-on device
	pwmMask = 0
	tachinMask = 0
	outState = 0
	for ch = 1, PWM_COUNT do
		pwmOn[ch] = 0
		pwmOff[ch] = 0
	end
	adcIntervalMs = DEFAULT_BROADCAST_MS
	tachIntervalMs = DEFAULT_BROADCAST_MS
	applyOutputs()
	adcTimer:reset()
	tachTimer:reset()

	local whoami = {0, 0, 0, 0, 0, 0, 0, 0}
	setByte(whoami, 0, 1)   -- version, arbitrary
	setTwoBytesMsb(whoami, 4, PWM_CLOCK_PERIOD)
	setTwoBytesMsb(whoami, 6, TACHIN_CLOCK_PERIOD)
	txCan(bus, baseId + 8, isExtId, whoami)
end

function onConfig(bus, id, dlc, data)
	pwmMask = getByte(data, 0)
	tachinMask = getByte(data, 2)
	adcIntervalMs = getByte(data, 4)
	tachIntervalMs = getByte(data, 5)
	if adcIntervalMs == 0 then adcIntervalMs = DEFAULT_BROADCAST_MS end
	if tachIntervalMs == 0 then tachIntervalMs = DEFAULT_BROADCAST_MS end

	print('MSIO: config pwm_mask=' .. pwmMask .. ' tachin_mask=' .. tachinMask ..
		' adcMs=' .. adcIntervalMs .. ' tachMs=' .. tachIntervalMs)
	applyOutputs()
end

-- shared parser for the two-PWM-channel packets (+2, +3, +4): firstChannel
-- is 0-based (0 for +2/PWM1,2; 2 for +3/PWM3,4; 4 for +4/PWM5,6)
function onPwmPair(firstChannel, data)
	pwmOn[firstChannel + 1] = getTwoBytesMsb(data, 0)
	pwmOff[firstChannel + 1] = getTwoBytesMsb(data, 2)
	pwmOn[firstChannel + 2] = getTwoBytesMsb(data, 4)
	pwmOff[firstChannel + 2] = getTwoBytesMsb(data, 6)
	applyOutputs()
end

function onPwm12(bus, id, dlc, data)
	onPwmPair(0, data)
end

function onPwm34(bus, id, dlc, data)
	onPwmPair(2, data)
end

function onPwm56(bus, id, dlc, data)
	onPwmPair(4, data)
end

function onPwm7AndOutState(bus, id, dlc, data)
	pwmOn[7] = getTwoBytesMsb(data, 0)
	pwmOff[7] = getTwoBytesMsb(data, 2)
	outState = getByte(data, 4)
	applyOutputs()
end

canRxAdd(bus, baseId + 0, onPing)
canRxAdd(bus, baseId + 1, onConfig)
canRxAdd(bus, baseId + 2, onPwm12)
canRxAdd(bus, baseId + 3, onPwm34)
canRxAdd(bus, baseId + 4, onPwm56)
canRxAdd(bus, baseId + 5, onPwm7AndOutState)

------------------------------------------------------------
-- device -> ECU telemetry (simulated analog/switch/tach data - this device
-- does not read any real sensor, it just needs to produce plausible,
-- changing values so the ECU-side decode can be observed)
------------------------------------------------------------

local simTick = 0

-- triangle wave 0..1023 (10-bit, matching the real device's ADC range),
-- each channel using a different period/phase so the seven traces are
-- visually distinct in a logger/console
function simAdc(channelIndex)
	local period = 200 + channelIndex * 23
	local phase = (simTick + channelIndex * 30) % period
	local half = period / 2
	local ramp
	if phase < half then
		ramp = phase / half
	else
		ramp = 2 - (phase / half)
	end
	return math.floor(ramp * 1023)
end

-- switch inputs 1..3 (CANIN1..3): toggle slowly so state changes are visible
function simSwitchBit(switchIndex)
	local period = 100 + switchIndex * 40
	return math.floor(simTick / period) % 2
end

function sendAdcFrames()
	local adc14 = {0, 0, 0, 0, 0, 0, 0, 0}
	setTwoBytesMsb(adc14, 0, simAdc(0))
	setTwoBytesMsb(adc14, 2, simAdc(1))
	setTwoBytesMsb(adc14, 4, simAdc(2))
	setTwoBytesMsb(adc14, 6, simAdc(3))
	txCan(bus, baseId + 9, isExtId, adc14)

	local inputs = simSwitchBit(0) | (simSwitchBit(1) << 1) | (simSwitchBit(2) << 2)
	local adc57 = {0, 0, 0, 0, 0, 0, 0, 0}
	setByte(adc57, 0, inputs)
	setTwoBytesMsb(adc57, 2, simAdc(4))
	setTwoBytesMsb(adc57, 4, simAdc(5))
	setTwoBytesMsb(adc57, 6, simAdc(6))
	txCan(bus, baseId + 10, isExtId, adc57)
end

-- simulated wheel speed: fixed tooth count per broadcast, slowly varying
-- period, in the device's own tach-clock ticks (see TACHIN_CLOCK_PERIOD)
function sendTachFrame(tachIndex)
	local teeth = 4
	local periodTicks = 10000 + (simTick % 5000)

	local tach = {0, 0, 0, 0, 0, 0, 0, 0}
	setFourBytesMsb(tach, 0, periodTicks)
	setTwoBytesMsb(tach, 4, teeth)
	totalTeeth[tachIndex + 1] = (totalTeeth[tachIndex + 1] + teeth) & 0xffff
	setTwoBytesMsb(tach, 6, totalTeeth[tachIndex + 1])
	txCan(bus, baseId + 11 + tachIndex, isExtId, tach)
end

function onTick()
	simTick = simTick + 1

	if adcTimer:getElapsedSeconds() * 1000 > adcIntervalMs then
		adcTimer:reset()
		sendAdcFrames()
	end

	if tachTimer:getElapsedSeconds() * 1000 > tachIntervalMs then
		tachTimer:reset()
		for tachIndex = 0, 3 do
			if (tachinMask & (1 << tachIndex)) ~= 0 then
				sendTachFrame(tachIndex)
			end
		end
	end
end
