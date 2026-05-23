#define CAN0_MESSAGE_RAM_SIZE (4352)
#define CAN1_MESSAGE_RAM_SIZE (0)

#include <Arduino.h>
#include <ACANFD_SAME.h>
#include "CANMessageSignal.h"
#include "DeviceSignalsByteBit.h"

using namespace CANMessageSignal;

static const uint32_t CAN_BAUD = 500UL * 1000UL;
static const uint32_t SERIAL_BAUD = 115200UL;

// Terminal dashboard redraw rate.
static const uint32_t DISPLAY_REFRESH_MS = 100UL;

// Set to 1 for ANSI / VT100 terminals such as PuTTY, Tera Term, minicom, etc. This is the prefered option, produces a much cleaner serial output.
// Set to 0 for the Arduino Serial Monitor or other terminals that show escape characters. Only use if you have to use the Arduino serial monitor.
#define USE_ANSI_TERMINAL 1

static uint32_t gLastDisplayMs = 0;

static char gLastErrorLine[180] = "";

static void beginDashboardUpdate() {
#if USE_ANSI_TERMINAL
  // ANSI live-view update without clearing the whole terminal.
  // This greatly reduces flicker compared with "\033[2J\033[H".
  Serial.print("\033[?25l");  // Hide cursor.
  Serial.print("\033[H");     // Move cursor to row 1, column 1.
#else
  Serial.println();
#endif
}

static void endDashboardUpdate() {
#if USE_ANSI_TERMINAL
  Serial.print("\033[J");  // Clear any old lines below the dashboard.
#endif
}

static void printDashboardNewline() {
#if USE_ANSI_TERMINAL
  Serial.print("\033[K\r\n");  // Clear rest of line, then move to next line.
#else
  Serial.print("\r\n");
#endif
}

static void printInt64(int64_t value) {
  if (value < 0) {
    Serial.print('-');
    value = -value;
  }

  char buf[22];
  uint8_t pos = sizeof(buf) - 1U;
  buf[pos] = '\0';

  uint64_t v = uint64_t(value);
  do {
    buf[--pos] = char('0' + (v % 10U));
    v /= 10U;
  } while (v > 0U && pos > 0U);

  Serial.print(&buf[pos]);
}

static void printSignalLine(const CanSignal& signal, bool invertDisplay = false) {
  Serial.print(signal.name());
  Serial.print(" = ");

  if (!signal.hasReceivedValue()) {
    Serial.print("no data yet");
    printDashboardNewline();
    return;
  }

  const char* label = signal.enumLabel();
  if (label != nullptr) {
    Serial.print(label);
    Serial.print(" raw=");
    printInt64(signal.rawSignalValue());
    printDashboardNewline();
    return;
  }

  double displayValue = signal.signalValue();
  if (invertDisplay) {
    displayValue = (displayValue == 0.0) ? 1.0 : 0.0;
  }

  Serial.print(displayValue, 3);

  const char* unit = signal.config().unit;
  if (unit != nullptr && unit[0] != '\0' && strcmp(unit, "NA") != 0) {
    Serial.print(" ");
    Serial.print(unit);
  }

  Serial.print(" raw=");
  printInt64(signal.rawSignalValue());
  printDashboardNewline();
}

static void storeDecodeError(const char* context, const CanMessage& message, const CANFDMessage& frame, const char* errorText) {
  snprintf(gLastErrorLine, sizeof(gLastErrorLine),
           "%s decode failed for %s id=0x%lX len=%u error=%s",
           context,
           message.name(),
           static_cast<unsigned long>(frame.id),
           unsigned(frame.len),
           errorText ? errorText : "unknown");
}

static void printLiveDashboard() {
  beginDashboardUpdate();

  Serial.print("CANMessageSignal RX signal live view");
  printDashboardNewline();
  printDashboardNewline();

  printSignalLine(rpm);
  printSignalLine(speedo);
  printSignalLine(rawECT);
  printSignalLine(gaugeECT);
  printSignalLine(oilGauge);
  printSignalLine(celOn);
  printSignalLine(celFlashing);
  printSignalLine(batteryLight);
  printSignalLine(oilLight);
  printSignalLine(coolantLight);
  printSignalLine(cruiseLight);
  printSignalLine(cruiseMainLight);
  printSignalLine(steeringLight);
  printSignalLine(absLight);
  printSignalLine(brakeWarnLight);
  printSignalLine(DscOffLightn, true);  // Active-low: raw 1 means the light is off.
  printSignalLine(tcLight);
  printSignalLine(tcFlashingLight);
  printSignalLine(dscTcEnable);

  if (gLastErrorLine[0] != '\0') {
    printDashboardNewline();
    Serial.print("Last decode error: ");
    Serial.print(gLastErrorLine);
    printDashboardNewline();
  }

  endDashboardUpdate();
}

static void serviceDisplay() {
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - gLastDisplayMs) >= DISPLAY_REFRESH_MS) {
    gLastDisplayMs = nowMs;
    printLiveDashboard();
  }
}

static bool configureCan0NoTx() {
  pinMode(PIN_CAN0_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN0_STANDBY, LOW);

  ACANFD_SAME_Settings settings(
    ACANFD_SAME_Settings::CLOCK_48MHz,
    CAN_BAUD,
    DataBitRateFactor::x1);

  settings.mModuleMode = ACANFD_SAME_Settings::NORMAL_FD;
  settings.mHardwareRxFIFO0Size = 64;
  settings.mDriverReceiveFIFO0Size = 64;
  settings.mHardwareRxFIFO1Size = 64;
  settings.mDriverReceiveFIFO1Size = 64;
  settings.mHardwareRxFIFO0Payload = ACANFD_SAME_Settings::PAYLOAD_64_BYTES;
  settings.mHardwareRxFIFO1Payload = ACANFD_SAME_Settings::PAYLOAD_64_BYTES;
  settings.mNonMatchingStandardFrameReception = ACANFD_SAME_FilterAction::FIFO0;
  settings.mNonMatchingExtendedFrameReception = ACANFD_SAME_FilterAction::FIFO1;

  const uint32_t err = can0.beginFD(settings);
  Serial.print("can0 begin ok=");
  Serial.print(err == 0 ? "yes" : "no");
  Serial.print(" status=0x");
  Serial.println(err, HEX);
  return err == 0;
}

static void printRegistrationResult(bool ok) {
  Serial.print("signal registration ok=");
  Serial.println(ok ? "yes" : "no");
}

static bool decodeOneMessage(CanMessage& message, const CANFDMessage& rx) {
  if (!message.matches(rx)) {
    return false;
  }

  if (!message.decode(rx) && message.hasError()) {
    storeDecodeError("RX polling", message, rx, message.errorText());
    message.clearError();
  }

  return true;
}

static void decodeKnownMessages(const CANFDMessage& rx) {
  if (decodeOneMessage(engine201, rx)) return;
  if (decodeOneMessage(engine240, rx)) return;
  if (decodeOneMessage(engine420, rx)) return;
  if (decodeOneMessage(engine650, rx)) return;
  if (decodeOneMessage(eps300, rx)) return;
  if (decodeOneMessage(abs212, rx)) return;
  // Unknown frames are normal on a real CAN bus. Ignore silently.
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 1500U)) {
    delay(10);
  }

  Serial.println("CANMessageSignal RX manual polling signal live-view example");
  Serial.println("Use an ANSI-capable serial terminal. Arduino Serial Monitor will not clear the console correctly.");

  printRegistrationResult(setDeviceSignals());
  (void)configureCan0NoTx();
}

void loop() {
  CANFDMessage FD0rx;
  CANFDMessage FD1rx;

  while (can0.receiveFD0(FD0rx)) {
    decodeKnownMessages(FD0rx);
  }

  while (can0.receiveFD1(FD0rx)) {
    decodeKnownMessages(FD0rx);
  }

  serviceDisplay();
}
