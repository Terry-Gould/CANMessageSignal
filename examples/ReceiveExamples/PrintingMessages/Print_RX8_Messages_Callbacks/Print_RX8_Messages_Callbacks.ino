#define CAN0_MESSAGE_RAM_SIZE (4352)
#define CAN1_MESSAGE_RAM_SIZE (0)

#include <Arduino.h>
#include <ACANFD_SAME.h>
#include "CANMessageSignal.h"
#include "DeviceSignalsByteBit.h"

using namespace CANMessageSignal;

static const uint32_t CAN_BAUD = 500UL * 1000UL;
static const uint32_t SERIAL_BAUD = 115200UL;

// Set to 1 for ANSI / VT100 terminals such as PuTTY, Tera Term, minicom, etc. This is the prefered option, produces a much cleaner serial output.
// Set to 0 for the Arduino Serial Monitor or other terminals that show escape characters. Only use if you have to use the Arduino serial monitor.
#define USE_ANSI_TERMINAL 1

// Terminal dashboard redraw rate.
static const uint32_t DISPLAY_REFRESH_MS = 100UL;

static uint32_t gLastDisplayMs = 0;

static CANFDMessage gFrameEngine201;
static CANFDMessage gFrameEngine240;
static CANFDMessage gFrameEngine420;
static CANFDMessage gFrameEngine650;
static CANFDMessage gFrameEps300;
static CANFDMessage gFrameAbs212;

static bool gSeenEngine201 = false;
static bool gSeenEngine240 = false;
static bool gSeenEngine420 = false;
static bool gSeenEngine650 = false;
static bool gSeenEps300 = false;
static bool gSeenAbs212 = false;

static char gLastErrorLine[180] = "";

static void beginDashboardUpdate() {
#if USE_ANSI_TERMINAL
  // ANSI live-view update without clearing the whole terminal.
  // This greatly reduces flicker compared with "\033[2J\033[H".
  Serial.print("\033[?25l"); // Hide cursor.
  Serial.print("\033[H");    // Move cursor to row 1, column 1.
#else
  Serial.println();
  Serial.println("----------------------------------------");
#endif
}

static void endDashboardUpdate() {
#if USE_ANSI_TERMINAL
  Serial.print("\033[J");    // Clear any old lines below the dashboard.
#endif
}

static void printDashboardNewline() {
#if USE_ANSI_TERMINAL
  Serial.print("\033[K\r\n"); // Clear rest of line, then move to next line.
#else
  Serial.println();
#endif
}

static const char* frameTypeText(const CANFDMessage& frame) {
  switch (frame.type) {
    case CANFDMessage::CAN_DATA:
      return "CAN_DATA";
    case CANFDMessage::CAN_REMOTE:
      return "CAN_REMOTE";
    case CANFDMessage::CANFD_NO_BIT_RATE_SWITCH:
      return "CANFD_NO_BRS";
    case CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH:
      return "CANFD_BRS";
    default:
      return "UNKNOWN";
  }
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

static void printCanFrameHeader(const CANFDMessage& frame) {
  Serial.print(" id=0x");
  Serial.print(frame.id, HEX);
  Serial.print(frame.ext ? " EXT" : " STD");
  Serial.print(" len=");
  Serial.print(frame.len);
  Serial.print(" type=");
  Serial.print(frameTypeText(frame));
}

static void printSignalValue(const CanSignal& signal) {
  Serial.print(signal.name());
  Serial.print("=");

  // DSC_OffLight is active-low in DeviceSignalsByteBit.h: raw 1 means the lamp is off,
  // raw 0 means the lamp is on. Display the logical lamp state for readability.
  if (&signal == &DscOffLightn) {
    const int64_t raw = signal.rawSignalValue();
    const double logicalValue = (raw == 0) ? 1.0 : 0.0;
    Serial.print(logicalValue, 3);
    Serial.print(" raw=");
    printInt64(raw);
    return;
  }

  const char* label = signal.enumLabel();
  if (label != nullptr) {
    Serial.print(label);
    Serial.print("(");
    printInt64(signal.rawSignalValue());
    Serial.print(")");
  } else {
    Serial.print(signal.signalValue(), 3);
    Serial.print(" raw=");
    printInt64(signal.rawSignalValue());
  }
}

static void printMessageLine(const CanMessage& message, const CANFDMessage& frame, bool seen) {
  Serial.print("RX ");
  Serial.print(message.name());

  if (!seen) {
    Serial.print(": no data yet");
    printDashboardNewline();
    return;
  }

  printCanFrameHeader(frame);
  Serial.print(" : ");

  bool printedAny = false;
  for (uint8_t i = 0; i < message.signalCount(); i++) {
    const CanSignal* signal = message.signalAt(i);
    if (signal == nullptr) {
      continue;
    }

    if (printedAny) {
      Serial.print(", ");
    }

    printSignalValue(*signal);
    printedAny = true;
  }

  printDashboardNewline();
}

static void markDecodedMessage(const CanMessage& message, const CANFDMessage& frame) {
  if (&message == &engine201) {
    gFrameEngine201 = frame;
    gSeenEngine201 = true;
  } else if (&message == &engine240) {
    gFrameEngine240 = frame;
    gSeenEngine240 = true;
  } else if (&message == &engine420) {
    gFrameEngine420 = frame;
    gSeenEngine420 = true;
  } else if (&message == &engine650) {
    gFrameEngine650 = frame;
    gSeenEngine650 = true;
  } else if (&message == &eps300) {
    gFrameEps300 = frame;
    gSeenEps300 = true;
  } else if (&message == &abs212) {
    gFrameAbs212 = frame;
    gSeenAbs212 = true;
  }
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

  Serial.print("CANMessageSignal RX live view");
  printDashboardNewline();
  printDashboardNewline();

  printMessageLine(engine201, gFrameEngine201, gSeenEngine201);
  printMessageLine(engine240, gFrameEngine240, gSeenEngine240);
  printMessageLine(engine420, gFrameEngine420, gSeenEngine420);
  printMessageLine(engine650, gFrameEngine650, gSeenEngine650);
  printMessageLine(eps300, gFrameEps300, gSeenEps300);
  printMessageLine(abs212, gFrameAbs212, gSeenAbs212);

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


static void printRegistrationResult(bool ok) {
  Serial.print("signal registration ok=");
  Serial.println(ok ? "yes" : "no");
}

static void decodeCallbackMessage(CanMessage& message, const CANFDMessage& inMessage) {
  if (message.decode(inMessage)) {
    markDecodedMessage(message, inMessage);
  } else if (message.hasError()) {
    storeDecodeError("RX callback", message, inMessage, message.errorText());
    message.clearError();
  }
}

static void callbackEngine201(const CANFDMessage& inMessage) { decodeCallbackMessage(engine201, inMessage); }
static void callbackEngine240(const CANFDMessage& inMessage) { decodeCallbackMessage(engine240, inMessage); }
static void callbackEngine420(const CANFDMessage& inMessage) { decodeCallbackMessage(engine420, inMessage); }
static void callbackEngine650(const CANFDMessage& inMessage) { decodeCallbackMessage(engine650, inMessage); }
static void callbackEps300(const CANFDMessage& inMessage) { decodeCallbackMessage(eps300, inMessage); }
static void callbackAbs212(const CANFDMessage& inMessage) { decodeCallbackMessage(abs212, inMessage); }

static bool configureCan0WithCallbacks() {
  pinMode(PIN_CAN0_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN0_STANDBY, LOW);

  ACANFD_SAME_Settings settings(
    ACANFD_SAME_Settings::CLOCK_48MHz,
    CAN_BAUD,
    DataBitRateFactor::x1
  );

  settings.mModuleMode = ACANFD_SAME_Settings::NORMAL_FD;
  settings.mHardwareRxFIFO0Size = 64;
  settings.mDriverReceiveFIFO0Size = 64;
  settings.mHardwareRxFIFO0Payload = ACANFD_SAME_Settings::PAYLOAD_64_BYTES;
  settings.mNonMatchingStandardFrameReception = ACANFD_SAME_FilterAction::REJECT;
  settings.mNonMatchingExtendedFrameReception = ACANFD_SAME_FilterAction::REJECT;

  ACANFD_SAME::StandardFilters standardFilters;
  bool filtersOk = true;
  filtersOk &= standardFilters.addSingle(0x201, ACANFD_SAME_FilterAction::FIFO0, callbackEngine201);
  filtersOk &= standardFilters.addSingle(0x240, ACANFD_SAME_FilterAction::FIFO0, callbackEngine240);
  filtersOk &= standardFilters.addSingle(0x420, ACANFD_SAME_FilterAction::FIFO0, callbackEngine420);
  filtersOk &= standardFilters.addSingle(0x650, ACANFD_SAME_FilterAction::FIFO0, callbackEngine650);
  filtersOk &= standardFilters.addSingle(0x300, ACANFD_SAME_FilterAction::FIFO0, callbackEps300);
  filtersOk &= standardFilters.addSingle(0x212, ACANFD_SAME_FilterAction::FIFO0, callbackAbs212);

  Serial.print("standard filters ok=");
  Serial.print(filtersOk ? "yes" : "no");
  Serial.print(" count=");
  Serial.println(standardFilters.count());

  const uint32_t err = can0.beginFD(settings, standardFilters);
  Serial.print("can0 begin ok=");
  Serial.print(err == 0 ? "yes" : "no");
  Serial.print(" status=0x");
  Serial.println(err, HEX);
  return err == 0;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 1500U)) {
    delay(10);
  }

  Serial.println("CANMessageSignal 1.1.1 RX callback live-view example");
  #if USE_ANSI_TERMINAL
  Serial.println("ANSI terminal live-view enabled.");
#else
  Serial.println("Arduino Serial Monitor compatible output enabled.");
#endif

  printRegistrationResult(setDeviceSignals());
  (void)configureCan0WithCallbacks();
}

void loop() {
  // ACANFD invokes the filter callbacks when dispatchReceivedMessage() drains
  // FIFO0. The received CANFDMessage is passed straight to the matching callback.
  while (can0.dispatchReceivedMessage()) {
    // All decoding happens in the matching callback.
  }

  serviceDisplay();
}
