#define CAN0_MESSAGE_RAM_SIZE (1728)
#define CAN1_MESSAGE_RAM_SIZE (0)

#include <Arduino.h>
#include <ACANFD_SAME.h>
#include "CANMessageSignal.h"
#include "DeviceSignalsByteBit.h"

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
// Raw/gauge ECT independent temperature cycle
// -12 -> 150 over 1 minute
// dwell at 150 for 1 minute
// gaugeECT only = raw 0xFF for 20 seconds
// 150 -> -12 over 1 minute
// dwell at -12 for 1 minute
// gaugeECT only = raw 0xFF for 20 seconds
// repeat forever
// -----------------------------------------------------------------------------
static const double ECT_MIN_C = -12.0;
static const double ECT_MAX_C = 150.0;
static const double GAUGE_ECT_RAW_FF_PHYSICAL = 215.0;  // raw = physical - offset = 215 - (-40) = 255 = 0xFF

static const uint32_t ECT_RAMP_MS = 60000UL;
static const uint32_t ECT_DWELL_MS = 60000UL;
static const uint32_t ECT_FAULT_MS = 20000UL;

enum EctPhase : uint8_t {
  ECT_RAMP_UP,
  ECT_DWELL_MAX,
  ECT_GAUGE_FF_MAX,
  ECT_RAMP_DOWN,
  ECT_DWELL_MIN,
  ECT_GAUGE_FF_MIN
};

static EctPhase gEctPhase = ECT_RAMP_UP;
static uint32_t gEctPhaseStartMs = 0;

static void printEctPhaseEvent(const char* text) {
  Serial.print("ECT: ");
  Serial.println(text);
}

static void advanceEctPhase(uint32_t nowMs) {
  switch (gEctPhase) {
    case ECT_RAMP_UP:
      gEctPhase = ECT_DWELL_MAX;
      gEctPhaseStartMs = nowMs;
      printEctPhaseEvent("ramp up ended, max dwell started");
      break;

    case ECT_DWELL_MAX:
      gEctPhase = ECT_GAUGE_FF_MAX;
      gEctPhaseStartMs = nowMs;
      printEctPhaseEvent("max dwell ended, gaugeECT raw 0xFF started");
      break;

    case ECT_GAUGE_FF_MAX:
      gEctPhase = ECT_RAMP_DOWN;
      gEctPhaseStartMs = nowMs;
      printEctPhaseEvent("gaugeECT raw 0xFF ended, ramp down started");
      break;

    case ECT_RAMP_DOWN:
      gEctPhase = ECT_DWELL_MIN;
      gEctPhaseStartMs = nowMs;
      printEctPhaseEvent("ramp down ended, min dwell started");
      break;

    case ECT_DWELL_MIN:
      gEctPhase = ECT_GAUGE_FF_MIN;
      gEctPhaseStartMs = nowMs;
      printEctPhaseEvent("min dwell ended, gaugeECT raw 0xFF started");
      break;

    case ECT_GAUGE_FF_MIN:
    default:
      gEctPhase = ECT_RAMP_UP;
      gEctPhaseStartMs = nowMs;
      printEctPhaseEvent("gaugeECT raw 0xFF ended, ramp up started");
      break;
  }
}

static void updateEctCycle(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - gEctPhaseStartMs;
  double rawTempC = ECT_MIN_C;
  double gaugeTempC = ECT_MIN_C;

  switch (gEctPhase) {
    case ECT_RAMP_UP:
      {
        if (elapsed >= ECT_RAMP_MS) {
          advanceEctPhase(nowMs);
          rawTempC = ECT_MAX_C;
          gaugeTempC = ECT_MAX_C;
        } else {
          const double f = double(elapsed) / double(ECT_RAMP_MS);
          rawTempC = ECT_MIN_C + f * (ECT_MAX_C - ECT_MIN_C);
          gaugeTempC = rawTempC;
        }
        break;
      }

    case ECT_DWELL_MAX:
      rawTempC = ECT_MAX_C;
      gaugeTempC = ECT_MAX_C;
      if (elapsed >= ECT_DWELL_MS) {
        advanceEctPhase(nowMs);
        gaugeTempC = GAUGE_ECT_RAW_FF_PHYSICAL;
      }
      break;

    case ECT_GAUGE_FF_MAX:
      rawTempC = ECT_MAX_C;
      gaugeTempC = GAUGE_ECT_RAW_FF_PHYSICAL;
      if (elapsed >= ECT_FAULT_MS) {
        advanceEctPhase(nowMs);
        rawTempC = ECT_MAX_C;
        gaugeTempC = ECT_MAX_C;
      }
      break;

    case ECT_RAMP_DOWN:
      {
        if (elapsed >= ECT_RAMP_MS) {
          advanceEctPhase(nowMs);
          rawTempC = ECT_MIN_C;
          gaugeTempC = ECT_MIN_C;
        } else {
          const double f = double(elapsed) / double(ECT_RAMP_MS);
          rawTempC = ECT_MAX_C - f * (ECT_MAX_C - ECT_MIN_C);
          gaugeTempC = rawTempC;
        }
        break;
      }

    case ECT_DWELL_MIN:
      rawTempC = ECT_MIN_C;
      gaugeTempC = ECT_MIN_C;
      if (elapsed >= ECT_DWELL_MS) {
        advanceEctPhase(nowMs);
        gaugeTempC = GAUGE_ECT_RAW_FF_PHYSICAL;
      }
      break;

    case ECT_GAUGE_FF_MIN:
    default:
      rawTempC = ECT_MIN_C;
      gaugeTempC = GAUGE_ECT_RAW_FF_PHYSICAL;
      if (elapsed >= ECT_FAULT_MS) {
        advanceEctPhase(nowMs);
        rawTempC = ECT_MIN_C;
        gaugeTempC = ECT_MIN_C;
      }
      break;
  }

  rawECT.setSignalValue(rawTempC);
  gaugeECT.setSignalValue(gaugeTempC);
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

  gEctPhaseStartMs = millis();
  gSeqStateStartMs = millis();

  printEctPhaseEvent("ramp up started");
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