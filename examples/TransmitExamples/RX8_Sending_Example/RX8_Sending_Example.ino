#define CAN0_MESSAGE_RAM_SIZE (1728)
#define CAN1_MESSAGE_RAM_SIZE (0)

#include <Arduino.h>
#include <ACANFD_SAME.h>
#include "CANMessageSignal.h"
#include "DeviceSignalsByteBit.h"

// Required only when TinyUSB is selected, so the external Adafruit TinyUSB Library provides USB Serial.
#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>
#endif

using namespace CANMessageSignal;

CanChannel microCan0(can0);

// -----------------------------------------------------------------------------
// Sequential warning-lamp / enum test cycle
// This runs independently from RPM/speed/ECT.
// -----------------------------------------------------------------------------
enum SequenceState : uint8_t {
  SEQ_OIL_LOW,
  SEQ_OIL_OK,
  SEQ_OIL_FAULT,

  SEQ_CEL_ON,
  SEQ_CEL_OFF_WAIT,
  SEQ_CEL_FLASH_ON,
  SEQ_CEL_FLASH_OFF_WAIT,
  SEQ_BATTERY_ON,
  SEQ_BATTERY_OFF_WAIT,
  SEQ_OIL_LIGHT_ON,
  SEQ_OIL_LIGHT_OFF_WAIT,
  SEQ_COOLANT_ON,
  SEQ_COOLANT_OFF_WAIT,
  SEQ_CRUISE_ON,
  SEQ_CRUISE_OFF_WAIT,
  SEQ_CRUISE_MAIN_ON,
  SEQ_CRUISE_MAIN_OFF_WAIT,
  SEQ_STEERING_ON,
  SEQ_STEERING_OFF_WAIT,
  SEQ_ABS_ON,
  SEQ_ABS_OFF_WAIT,
  SEQ_BRAKE_WARN_ON,
  SEQ_BRAKE_WARN_OFF_WAIT,
  SEQ_DSC_OFF_ON,
  SEQ_DSC_OFF_OFF_WAIT,
  SEQ_TC_ON,
  SEQ_TC_OFF_WAIT,
  SEQ_TC_FLASH_ON,
  SEQ_TC_FLASH_OFF_WAIT,

  SEQ_COUNT
};

static uint32_t sequenceDurationMs(SequenceState state);
static void applySequenceState(SequenceState state);
static void advanceSequence(uint32_t nowMs);
static void updateSequence(uint32_t nowMs);

// -----------------------------------------------------------------------------
// Speed/RPM ramp
// 0 -> max over 5 s, hold max 1 s, max -> 0 over 5 s, hold zero 1 s.
// -----------------------------------------------------------------------------
static double rampFraction(uint32_t nowMs) {
  const uint32_t rampUpMs = 5000UL;
  const uint32_t holdMaxMs = 1000UL;
  const uint32_t rampDownMs = 5000UL;
  const uint32_t holdZeroMs = 1000UL;

  const uint32_t periodMs = rampUpMs + holdMaxMs + rampDownMs + holdZeroMs;
  const uint32_t t = nowMs % periodMs;

  if (t < rampUpMs) {
    return double(t) / double(rampUpMs);
  }

  if (t < rampUpMs + holdMaxMs) {
    return 1.0;
  }

  if (t < rampUpMs + holdMaxMs + rampDownMs) {
    const uint32_t downT = t - rampUpMs - holdMaxMs;
    return 1.0 - (double(downT) / double(rampDownMs));
  }

  return 0.0;
}

// -----------------------------------------------------------------------------
// Raw/gauge ECT independent gauge sweep
// The RX-8 ECT gauge is non-linear: the needle only moves from 46-70 C and
// 111-140 C. The sweep skips the centre dead-band so the needle does not dwell
// near the midpoint.
// -----------------------------------------------------------------------------
static const double ECT_GAUGE_MIN_C = 46.0;
static const double ECT_GAUGE_LOWER_MOVE_END_C = 70.0;
static const double ECT_GAUGE_UPPER_MOVE_START_C = 111.0;
static const double ECT_GAUGE_MAX_C = 140.0;

static const uint32_t ECT_SWEEP_MS = 90000UL;
static const uint32_t ECT_DWELL_BOTTOM_MS = 5000UL;
static const uint32_t ECT_DWELL_TOP_MS = 5000UL;

enum EctSweepPhase : uint8_t {
  ECT_DWELL_BOTTOM,
  ECT_SWEEP_UP,
  ECT_DWELL_TOP,
  ECT_SWEEP_DOWN
};

static EctSweepPhase gEctSweepPhase = ECT_DWELL_BOTTOM;
static uint32_t gEctSweepPhaseStartMs = 0;

static void printEctPhaseEvent(const char* text) {
  Serial.print("ECT: ");
  Serial.println(text);
}

static double clamp01(double value) {
  if (value < 0.0) return 0.0;
  if (value > 1.0) return 1.0;
  return value;
}

static double ectGaugeSweepTempC(double needleFraction) {
  const double f = clamp01(needleFraction);

  if (f < 0.5) {
    const double lowerF = f / 0.5;
    return ECT_GAUGE_MIN_C +
           lowerF * (ECT_GAUGE_LOWER_MOVE_END_C - ECT_GAUGE_MIN_C);
  }

  const double upperF = (f - 0.5) / 0.5;
  return ECT_GAUGE_UPPER_MOVE_START_C +
         upperF * (ECT_GAUGE_MAX_C - ECT_GAUGE_UPPER_MOVE_START_C);
}

static void setEctSignals(double tempC) {
  rawECT.setSignalValue(tempC);
  gaugeECT.setSignalValue(tempC);
}

static void advanceEctSweepPhase(uint8_t nextPhase, uint32_t nowMs, const char* text) {
  gEctSweepPhase = EctSweepPhase(nextPhase);
  gEctSweepPhaseStartMs = nowMs;
  printEctPhaseEvent(text);
}

static void updateEctCycle(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - gEctSweepPhaseStartMs;

  switch (gEctSweepPhase) {
    case ECT_DWELL_BOTTOM:
      setEctSignals(ECT_GAUGE_MIN_C);
      if (elapsed >= ECT_DWELL_BOTTOM_MS) {
        advanceEctSweepPhase(ECT_SWEEP_UP, nowMs, "bottom dwell ended, sweep up started");
      }
      break;

    case ECT_SWEEP_UP:
      if (elapsed >= ECT_SWEEP_MS) {
        setEctSignals(ECT_GAUGE_MAX_C);
        advanceEctSweepPhase(ECT_DWELL_TOP, nowMs, "sweep up ended, top dwell started");
      } else {
        setEctSignals(ectGaugeSweepTempC(double(elapsed) / double(ECT_SWEEP_MS)));
      }
      break;

    case ECT_DWELL_TOP:
      setEctSignals(ECT_GAUGE_MAX_C);
      if (elapsed >= ECT_DWELL_TOP_MS) {
        advanceEctSweepPhase(ECT_SWEEP_DOWN, nowMs, "top dwell ended, sweep down started");
      }
      break;

    case ECT_SWEEP_DOWN:
    default:
      if (elapsed >= ECT_SWEEP_MS) {
        setEctSignals(ECT_GAUGE_MIN_C);
        advanceEctSweepPhase(ECT_DWELL_BOTTOM, nowMs, "sweep down ended, bottom dwell started");
      } else {
        const double f = 1.0 - (double(elapsed) / double(ECT_SWEEP_MS));
        setEctSignals(ectGaugeSweepTempC(f));
      }
      break;
  }
}

static SequenceState gSeqState = SEQ_OIL_LOW;
static uint32_t gSeqStateStartMs = 0;

static void allSequenceOutputsOff() {
  oilGauge.setSignalValue("Fault");  // Enum label -> raw 0

  celOn.setSignalValue(0);
  celFlashing.setSignalValue(0);
  coolantLight.setSignalValue(0);
  batteryLight.setSignalValue(0);
  oilLight.setSignalValue(0);
  cruiseLight.setSignalValue(0);
  cruiseMainLight.setSignalValue(0);
  steeringLight.setSignalValue(0);
  absLight.setSignalValue(0);
  brakeWarnLight.setSignalValue(0);
  DscOffLightn.setSignalValue(1);  // Inverted signal: 1 = off, 0 = on
  tcLight.setSignalValue(0);
  tcFlashingLight.setSignalValue(0);

  dscTcEnable.setSignalValue(1);  // Required for DSC/TC lamps
}

static uint32_t sequenceDurationMs(SequenceState state) {
  switch (state) {
    case SEQ_OIL_LOW:
    case SEQ_OIL_FAULT:
      return 6000UL;

    case SEQ_CEL_FLASH_ON:
    case SEQ_TC_FLASH_ON:
      return 5000UL;

    case SEQ_OIL_OK:
    case SEQ_CEL_OFF_WAIT:
    case SEQ_CEL_FLASH_OFF_WAIT:
    case SEQ_COOLANT_OFF_WAIT:
    case SEQ_BATTERY_OFF_WAIT:
    case SEQ_OIL_LIGHT_OFF_WAIT:
    case SEQ_CRUISE_OFF_WAIT:
    case SEQ_CRUISE_MAIN_OFF_WAIT:
    case SEQ_STEERING_OFF_WAIT:
    case SEQ_ABS_OFF_WAIT:
    case SEQ_BRAKE_WARN_OFF_WAIT:
    case SEQ_DSC_OFF_OFF_WAIT:
    case SEQ_TC_OFF_WAIT:
    case SEQ_TC_FLASH_OFF_WAIT:
      return 1000UL;

    default:
      return 2000UL;
  }
}

static void applySequenceState(SequenceState state) {
  allSequenceOutputsOff();

  switch (state) {
    case SEQ_OIL_LOW:
      oilGauge.setSignalValue("Low");
      Serial.println("SEQ: OilGauge = Low");
      break;

    case SEQ_OIL_OK:
      oilGauge.setSignalValue("Ok");
      Serial.println("SEQ: OilGauge = Ok");
      break;

    case SEQ_OIL_FAULT:
      oilGauge.setSignalValue("Fault");
      Serial.println("SEQ: OilGauge = Fault");
      break;

    case SEQ_CEL_ON:
      celOn.setSignalValue(1);
      Serial.println("SEQ: CheckEngineLight ON");
      break;

    case SEQ_CEL_OFF_WAIT:
      Serial.println("SEQ: CheckEngineLight OFF, wait 1s");
      break;

    case SEQ_CEL_FLASH_ON:
      celFlashing.setSignalValue(1);
      Serial.println("SEQ: FlashingEngineLight ON");
      break;

    case SEQ_CEL_FLASH_OFF_WAIT:
      Serial.println("SEQ: FlashingEngineLight OFF, wait 1s");
      break;

    case SEQ_COOLANT_ON:
      coolantLight.setSignalValue(1);
      Serial.println("SEQ: CoolantLight ON");
      break;

    case SEQ_COOLANT_OFF_WAIT:
      Serial.println("SEQ: CoolantLight OFF, wait 1s");
      break;

    case SEQ_BATTERY_ON:
      batteryLight.setSignalValue(1);
      Serial.println("SEQ: BatteryLight ON");
      break;

    case SEQ_BATTERY_OFF_WAIT:
      Serial.println("SEQ: BatteryLight OFF, wait 1s");
      break;

    case SEQ_OIL_LIGHT_ON:
      oilLight.setSignalValue(1);
      Serial.println("SEQ: OilLight ON");
      break;

    case SEQ_OIL_LIGHT_OFF_WAIT:
      Serial.println("SEQ: OilLight OFF, wait 1s");
      break;

    case SEQ_CRUISE_ON:
      cruiseLight.setSignalValue(1);
      Serial.println("SEQ: CruiseLight ON");
      break;

    case SEQ_CRUISE_OFF_WAIT:
      Serial.println("SEQ: CruiseLight OFF, wait 1s");
      break;

    case SEQ_CRUISE_MAIN_ON:
      cruiseMainLight.setSignalValue(1);
      Serial.println("SEQ: CruiseMainLight ON");
      break;

    case SEQ_CRUISE_MAIN_OFF_WAIT:
      Serial.println("SEQ: CruiseMainLight OFF, wait 1s");
      break;

    case SEQ_STEERING_ON:
      steeringLight.setSignalValue(1);
      Serial.println("SEQ: SteeringLight ON");
      break;

    case SEQ_STEERING_OFF_WAIT:
      Serial.println("SEQ: SteeringLight OFF, wait 1s");
      break;

    case SEQ_ABS_ON:
      absLight.setSignalValue(1);
      Serial.println("SEQ: ABSLight ON");
      break;

    case SEQ_ABS_OFF_WAIT:
      Serial.println("SEQ: ABSLight OFF, wait 1s");
      break;

    case SEQ_BRAKE_WARN_ON:
      brakeWarnLight.setSignalValue(1);
      Serial.println("SEQ: BrakeWarningLight ON");
      break;

    case SEQ_BRAKE_WARN_OFF_WAIT:
      Serial.println("SEQ: BrakeWarningLight OFF, wait 1s");
      break;

    case SEQ_DSC_OFF_ON:
      DscOffLightn.setSignalValue(0);  // Inverted: 0 = on
      dscTcEnable.setSignalValue(1);
      Serial.println("SEQ: DSC_OffLight ON");
      break;

    case SEQ_DSC_OFF_OFF_WAIT:
      DscOffLightn.setSignalValue(1);  // Inverted: 1 = off
      dscTcEnable.setSignalValue(1);
      Serial.println("SEQ: DSC_OffLight OFF, wait 1s");
      break;

    case SEQ_TC_ON:
      tcLight.setSignalValue(1);
      dscTcEnable.setSignalValue(1);
      Serial.println("SEQ: TractionControlLight ON");
      break;

    case SEQ_TC_OFF_WAIT:
      dscTcEnable.setSignalValue(1);
      Serial.println("SEQ: TractionControlLight OFF, wait 1s");
      break;

    case SEQ_TC_FLASH_ON:
      tcFlashingLight.setSignalValue(1);
      dscTcEnable.setSignalValue(1);
      Serial.println("SEQ: TractionControlFlashingLight ON");
      break;

    case SEQ_TC_FLASH_OFF_WAIT:
      dscTcEnable.setSignalValue(1);
      Serial.println("SEQ: TractionControlFlashingLight OFF, wait 1s");
      break;

    default:
      break;
  }
}

static void updateSequence(uint32_t nowMs) {
  if ((uint32_t)(nowMs - gSeqStateStartMs) >= sequenceDurationMs(gSeqState)) {
    gSeqState = SequenceState((uint8_t(gSeqState) + 1U) % uint8_t(SEQ_COUNT));
    gSeqStateStartMs = nowMs;
    applySequenceState(gSeqState);
  }
}

static void printCanError(const char* context, CanChannel& channel) {
  const uint32_t driverStatus = channel.lastDriverStatus();

  Serial.print(context);
  Serial.print(" error=");
  Serial.print(channel.errorText());

  if (driverStatus != 0U) {
    Serial.print(" driver=0x");
    Serial.print(driverStatus, HEX);
  }

  Serial.println();
}

static void sendAllMessages() {
  if (!microCan0.sendIfDue(engine201, 50)) {
    if (microCan0.lastDriverStatus() != 0U) printCanError("engine201", microCan0);
  }

  if (!microCan0.sendIfDue(engine240, 50)) {
    if (microCan0.lastDriverStatus() != 0U) printCanError("engine240", microCan0);
  }

  if (!microCan0.sendIfDue(engine420, 50)) {
    if (microCan0.lastDriverStatus() != 0U) printCanError("engine420", microCan0);
  }

  if (!microCan0.sendIfDue(engine650, 50)) {
    if (microCan0.lastDriverStatus() != 0U) printCanError("engine650", microCan0);
  }

  if (!microCan0.sendIfDue(eps300, 50)) {
    if (microCan0.lastDriverStatus() != 0U) printCanError("eps300", microCan0);
  }

  if (!microCan0.sendIfDue(abs212, 50)) {
    if (microCan0.lastDriverStatus() != 0U) printCanError("abs212", microCan0);
  }
}

void setup() {
  pinMode(PIN_CAN0_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN0_STANDBY, LOW);

  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 5000)) {
    delay(10);
  }

  Serial.println("CANMessageSignal RX8 example");
  Serial.println("Makes use of CanBitSignal, CanByteSignal and CanByteBitSignal");

  ACANFD_SAME_Settings settings(ACANFD_SAME_Settings::CLOCK_48MHz, 500 * 1000, DataBitRateFactor::x1);
  settings.mModuleMode = ACANFD_SAME_Settings::NORMAL_FD;

  const uint32_t beginStatus = can0.beginFD(settings);
  Serial.print("can0 begin ok=");
  Serial.print(beginStatus == 0 ? "yes" : "no");
  Serial.print(" status=0x");
  Serial.println(beginStatus, HEX);

  microCan0.setBusType(CAN_CLASSIC);

  if (!setDeviceSignals()) {
    Serial.println("Signal registration failed");
  } else {
    Serial.println("Signal registration ok");
  }

  gEctSweepPhaseStartMs = millis();
  gSeqStateStartMs = millis();

  printEctPhaseEvent("bottom dwell started");
  applySequenceState(gSeqState);
}

void loop() {
  const uint32_t now = millis();

  // Continuous RPM and vehicle speed sweep.
  const double f = rampFraction(now);
  rpm.setSignalValue(f * 8000.0);
  speedo.setSignalValue(f * 300.0);

  // Independent ECT cycle.
  updateEctCycle(now);

  // Independent sequential lamp/enum cycle.
  updateSequence(now);

  // Keep all RX-8 test messages alive.
  sendAllMessages();
}
