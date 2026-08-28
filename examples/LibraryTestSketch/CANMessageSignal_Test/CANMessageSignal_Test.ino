#define CAN0_MESSAGE_RAM_SIZE (1728)
#define CAN1_MESSAGE_RAM_SIZE (0)

#include <Arduino.h>
#include <ACANFD_SAME.h>
#include <string.h>

#include "CANMessageSignal.h"

// Required only when TinyUSB is selected, so the external Adafruit TinyUSB Library provides USB Serial.
#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>
#endif

using namespace CANMessageSignal;

CanSignal rpm({
  .name = "rpm",
  .dataType = UNSIGNED,
  .startBit = 7,
  .bitLength = 16,
  .endianness = BIG_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "rpm",
  .comment = "Engine speed",
  .min = 0,
  .max = 8000,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

CanMessage classicRpm({
  .name = "classicRpm",
  .idType = STANDARD,
  .id = 0x201,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "Classic CAN RPM frame"
});

CanMessage fdDlc9({
  .name = "fdDlc9",
  .idType = STANDARD,
  .id = 0x301,
  .dlc = 9, // Raw CAN FD DLC code 9 = 12 payload bytes.
  .defaultFill = 0xAA,
  .comment = "CAN FD DLC 9 mapping test",
  .frameFormat = FRAME_FD,
  .bitrateSwitch = false
});

CanMessage fdAutoBrs8({
  .name = "fdAutoBrs8",
  .idType = STANDARD,
  .id = 0x302,
  .dlc = 8, // DLC 8, but BRS true means FRAME_AUTO must select CAN FD.
  .defaultFill = 0x55,
  .comment = "CAN FD auto+BRS DLC 8 test",
  .frameFormat = FRAME_AUTO,
  .bitrateSwitch = true
});

CanMessage invalidClassicBrs({
  .name = "invalidClassicBrs",
  .idType = STANDARD,
  .id = 0x303,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "Classic CAN cannot use BRS",
  .frameFormat = FRAME_CLASSIC,
  .bitrateSwitch = true
});

CanSignal rawECTBig({
  .name = "RawECTBig",
  .dataType = UNSIGNED,
  .startBit = 31,
  .bitLength = 8,
  .endianness = BIG_ENDIAN,
  .factor = 1.0,
  .offset = -40.0,
  .unit = "degC",
  .comment = "8-bit big-endian byte-aligned ECT test",
  .min = -40,
  .max = 215,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

CanSignal rawECTLittle({
  .name = "RawECTLittle",
  .dataType = UNSIGNED,
  .startBit = 24,
  .bitLength = 8,
  .endianness = LITTLE_ENDIAN,
  .factor = 1.0,
  .offset = -40.0,
  .unit = "degC",
  .comment = "8-bit little-endian byte-aligned ECT test",
  .min = -40,
  .max = 215,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

CanSignal nibbleBig({
  .name = "NibbleBig",
  .dataType = UNSIGNED,
  .startBit = 23,
  .bitLength = 4,
  .endianness = BIG_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "",
  .comment = "4-bit big-endian bit-walk test",
  .min = 0,
  .max = 15,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

CanSignal nibbleLittle({
  .name = "NibbleLittle",
  .dataType = UNSIGNED,
  .startBit = 16,
  .bitLength = 4,
  .endianness = LITTLE_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "",
  .comment = "4-bit little-endian bit-walk test",
  .min = 0,
  .max = 15,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

EnumMap oilGaugeMap({
  {0, "Low"},
  {1, "Ok"},
  {2, "Fault"}
});

CanSignal oilGaugeEnum({
  .name = "OilGaugeEnum",
  .dataType = UNSIGNED,
  .startBit = 15,
  .bitLength = 8,
  .endianness = BIG_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "",
  .comment = "Enum label setSignalValue test",
  .min = 0,
  .max = 2,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = &oilGaugeMap,
  .overrideCapable = true
});

CanSignal noEnumSignal({
  .name = "NoEnumSignal",
  .dataType = UNSIGNED,
  .startBit = 23,
  .bitLength = 8,
  .endianness = BIG_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "",
  .comment = "Non-enum string set failure test",
  .min = 0,
  .max = 255,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

CanMessage endian8BigTest({
  .name = "endian8BigTest",
  .idType = STANDARD,
  .id = 0x401,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "8-bit big-endian signal should occupy byte 3"
});

CanMessage endian8LittleTest({
  .name = "endian8LittleTest",
  .idType = STANDARD,
  .id = 0x402,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "8-bit little-endian signal should occupy byte 3"
});

CanMessage endian4BigTest({
  .name = "endian4BigTest",
  .idType = STANDARD,
  .id = 0x403,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "4-bit big-endian signal should occupy high nibble of byte 2"
});

CanMessage endian4LittleTest({
  .name = "endian4LittleTest",
  .idType = STANDARD,
  .id = 0x404,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "4-bit little-endian signal should occupy low nibble of byte 2"
});

CanMessage enumLabelTest({
  .name = "enumLabelTest",
  .idType = STANDARD,
  .id = 0x405,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "Enum label setSignalValue test"
});

CanByteSignal byteBig16({
  .name = "ByteBig16",
  .dataType = UNSIGNED,
  .startByte = 0,
  .byteLength = 2,
  .endianness = BIG_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "",
  .comment = "CanByteSignal big-endian 16-bit test",
  .min = 0,
  .max = 65535,
  .defaultValue = 0,
  .overrideCapable = true
});

CanByteSignal byteLittle16({
  .name = "ByteLittle16",
  .dataType = UNSIGNED,
  .startByte = 0,
  .byteLength = 2,
  .endianness = LITTLE_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "",
  .comment = "CanByteSignal little-endian 16-bit test",
  .min = 0,
  .max = 65535,
  .defaultValue = 0,
  .overrideCapable = true
});

CanBitSignal bitAbsolute({
  .name = "BitAbsolute",
  .bit = 39,
  .comment = "CanBitSignal absolute bit 39 test",
  .defaultValue = false,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .inverted = false,
  .overrideCapable = true
});

CanByteBitSignal bitByte({
  .name = "BitByte",
  .byte = 2,
  .bit = 0,
  .comment = "CanByteBitSignal byte 2 bit 0 test",
  .defaultValue = false,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .inverted = false,
  .overrideCapable = true
});

CanBitSignal activeLowBit({
  .name = "ActiveLowBit",
  .bit = 12,
  .comment = "Inverted CanBitSignal test",
  .defaultValue = false,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .inverted = true,
  .overrideCapable = true
});

EnumMap bitOnOffMap({
  {0, "Off"},
  {1, "On"}
});

CanBitSignal enumBitSignal({
  .name = "EnumBitSignal",
  .bit = 5,
  .comment = "CanBitSignal enum label test",
  .defaultValue = false,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = &bitOnOffMap,
  .inverted = false,
  .overrideCapable = true
});

CanMessage byteBigTest({
  .name = "byteBigTest",
  .idType = STANDARD,
  .id = 0x501,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "CanByteSignal big-endian test"
});

CanMessage byteLittleTest({
  .name = "byteLittleTest",
  .idType = STANDARD,
  .id = 0x502,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "CanByteSignal little-endian test"
});

CanMessage bitAbsoluteTest({
  .name = "bitAbsoluteTest",
  .idType = STANDARD,
  .id = 0x503,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "CanBitSignal absolute bit test"
});

CanMessage bitByteTest({
  .name = "bitByteTest",
  .idType = STANDARD,
  .id = 0x504,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "CanByteBitSignal test"
});

CanMessage activeLowBitTest({
  .name = "activeLowBitTest",
  .idType = STANDARD,
  .id = 0x505,
  .dlc = 8,
  .defaultFill = 0xFF,
  .comment = "Inverted CanBitSignal test"
});

CanMessage enumBitTest({
  .name = "enumBitTest",
  .idType = STANDARD,
  .id = 0x506,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "CanBitSignal enum test"
});



CanSignal signedDecodeSignal({
  .name = "SignedDecodeSignal",
  .dataType = SIGNED,
  .startBit = 15,
  .bitLength = 16,
  .endianness = BIG_ENDIAN,
  .factor = 0.1,
  .offset = 0.0,
  .unit = "",
  .comment = "Signed decode test",
  .min = -3276.8,
  .max = 3276.7,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

CanMessage signedDecodeMessage({
  .name = "signedDecodeMessage",
  .idType = STANDARD,
  .id = 0x601,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "Signed decode message"
});

CanSignal floatDecodeSignal({
  .name = "FloatDecodeSignal",
  .dataType = FLOAT,
  .startBit = 0,
  .bitLength = 32,
  .endianness = LITTLE_ENDIAN,
  .factor = 1.0,
  .offset = 0.0,
  .unit = "",
  .comment = "Float decode test",
  .min = -1000,
  .max = 1000,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

CanMessage floatDecodeMessage({
  .name = "floatDecodeMessage",
  .idType = STANDARD,
  .id = 0x602,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "Float decode message"
});

CanMessageDatabase<8> rxDb;

CanChannel microCan0(can0);

static void printPayload(const uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
}

static void printPacked(const char* label, CanMessage& msg) {
  uint8_t payload[64];
  const bool ok = msg.pack(payload, sizeof(payload));

  Serial.print(label);
  Serial.print(" pack ok=");
  Serial.print(ok ? "yes" : "no");
  Serial.print(" dlc=");
  Serial.print(msg.dlc());
  Serial.print(" lengthBytes=");
  Serial.print(msg.lengthBytes());
  Serial.print(" payload=");
  if (ok) printPayload(payload, msg.lengthBytes());
  Serial.println();
}

static void printReceived(const char* label) {
  CANFDMessage rx;
  if (!can0.receiveFD0(rx) && !can0.receiveFD1(rx)) {
    Serial.print(label);
    Serial.println(" received=no");
    return;
  }

  Serial.print(label);
  Serial.print(" received=yes id=0x");
  Serial.print(rx.id, HEX);
  Serial.print(" len=");
  Serial.print(rx.len);
  Serial.print(" type=");
  Serial.print(int(rx.type));
  Serial.print(" data=");
  printPayload(rx.data, rx.len);
  Serial.println();
}

static void sendAndPrint(const char* label, CanMessage& msg) {
  const bool ok = microCan0.sendMessage(msg);
  Serial.print(label);
  Serial.print(" send ok=");
  Serial.print(ok ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(microCan0.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(microCan0.lastError()));
  Serial.print(" error=");
  Serial.print(microCan0.errorText());
  if (microCan0.lastDriverStatus() != 0) {
    Serial.print(" driver=0x");
    Serial.print(microCan0.lastDriverStatus(), HEX);
  }
  Serial.println();
}

static void printSendIfDueState(const char* label, bool sent) {
  Serial.print(label);
  Serial.print(" sendIfDue sent=");
  Serial.print(sent ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(microCan0.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(microCan0.lastError()));
  Serial.print(" error=");
  Serial.print(microCan0.errorText());
  if (microCan0.lastDriverStatus() != 0) {
    Serial.print(" driver=0x");
    Serial.print(microCan0.lastDriverStatus(), HEX);
  }
  Serial.println();
}

static void copyPayloadToCANFD(CANFDMessage& frame, CanMessage& msg) {
  uint8_t payload[64];
  memset(payload, 0, sizeof(payload));
  msg.pack(payload, sizeof(payload));
  frame.id = msg.id();
  frame.ext = (msg.idType() == EXTENDED);
  frame.idx = 0;
  frame.len = msg.lengthBytes();
  frame.type = msg.requiresCanFd() ? CANFDMessage::CANFD_NO_BIT_RATE_SWITCH : CANFDMessage::CAN_DATA;
  memcpy(frame.data, payload, frame.len);
}

static void printSignalRxState(const char* label, CanSignal& signal) {
  Serial.print(label);
  Serial.print(" value=");
  Serial.print(signal.signalValue(), 3);
  Serial.print(" raw=");
  Serial.print((long)signal.rawSignalValue());
  Serial.print(" hasReceived=");
  Serial.print(signal.hasReceivedValue() ? "yes" : "no");
  Serial.print(" lastUpdateMs=");
  Serial.print(signal.lastUpdateMs());
  const char* labelText = signal.enumLabel();
  if (labelText != nullptr) {
    Serial.print(" enumLabel=");
    Serial.print(labelText);
  }
  Serial.println();
}

static void printDbState(const char* label, bool ok) {
  Serial.print(label);
  Serial.print(" db decode ok=");
  Serial.print(ok ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(rxDb.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(rxDb.lastError()));
  Serial.print(" error=");
  Serial.print(rxDb.errorText());

  const CanMessage* msg = rxDb.lastDecodedMessage();
  if (msg != nullptr) {
    Serial.print(" decoded=");
    Serial.print(msg->name());
  }
  Serial.println();
}

static void runDecodeTests() {
  Serial.println("RX decode tests");

  Serial.print("add signedDecodeSignal ok=");
  Serial.print(signedDecodeMessage.addSignal(signedDecodeSignal) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(signedDecodeMessage.errorText());

  Serial.print("add floatDecodeSignal ok=");
  Serial.print(floatDecodeMessage.addSignal(floatDecodeSignal) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(floatDecodeMessage.errorText());

  Serial.print("rxDb add classicRpm ok=");
  Serial.print(rxDb.addMessage(classicRpm) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(rxDb.errorText());

  Serial.print("rxDb add enumLabelTest ok=");
  Serial.print(rxDb.addMessage(enumLabelTest) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(rxDb.errorText());

  Serial.print("rxDb add byteBigTest ok=");
  Serial.print(rxDb.addMessage(byteBigTest) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(rxDb.errorText());

  Serial.print("rxDb add signedDecodeMessage ok=");
  Serial.print(rxDb.addMessage(signedDecodeMessage) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(rxDb.errorText());

  Serial.print("rxDb add floatDecodeMessage ok=");
  Serial.print(rxDb.addMessage(floatDecodeMessage) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(rxDb.errorText());

  Serial.print("rxDb add duplicate classicRpm ok=");
  Serial.print(rxDb.addMessage(classicRpm) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(rxDb.errorText());
  rxDb.clearError();

  CANFDMessage rx;

  rpm.setSignalValue(4321);
  copyPayloadToCANFD(rx, classicRpm);
  Serial.print("classicRpm matches rx=");
  Serial.println(classicRpm.matches(rx) ? "yes" : "no");
  printDbState("classicRpm", rxDb.decode(rx));
  printSignalRxState("rpm after db decode expect 4321", rpm);

  oilGaugeEnum.setSignalValue("Fault");
  copyPayloadToCANFD(rx, enumLabelTest);
  printDbState("enumLabelTest", rxDb.decode(rx));
  printSignalRxState("oilGaugeEnum after db decode expect Fault", oilGaugeEnum);

  byteBig16.setSignalValue(0x1234);
  copyPayloadToCANFD(rx, byteBigTest);
  printDbState("byteBigTest", rxDb.decode(rx));
  printSignalRxState("byteBig16 after db decode expect 4660", byteBig16);

  signedDecodeSignal.setSignalValue(-123.4);
  copyPayloadToCANFD(rx, signedDecodeMessage);
  printDbState("signedDecodeMessage", rxDb.decode(rx));
  printSignalRxState("signedDecodeSignal after db decode expect -123.4", signedDecodeSignal);

  floatDecodeSignal.setSignalValue(12.5);
  copyPayloadToCANFD(rx, floatDecodeMessage);
  printDbState("floatDecodeMessage", rxDb.decode(rx));
  printSignalRxState("floatDecodeSignal after db decode expect 12.5", floatDecodeSignal);

  rx.id = 0x777;
  rx.ext = false;
  rx.type = CANFDMessage::CAN_DATA;
  rx.len = 8;
  memset(rx.data, 0, sizeof(rx.data));

  Serial.print("classicRpm direct no-match decode ok=");
  Serial.print(classicRpm.decode(rx) ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(classicRpm.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(classicRpm.lastError()));
  Serial.print(" error=");
  Serial.println(classicRpm.errorText());

  printDbState("unknown frame expect not decoded no error", rxDb.decode(rx));

  rx.id = classicRpm.id();
  rx.ext = false;
  rx.type = CANFDMessage::CAN_DATA;
  rx.len = 1;
  memset(rx.data, 0, sizeof(rx.data));

  printDbState("classicRpm short frame via db expect error", rxDb.decode(rx));
  rxDb.clearError();
  Serial.print("rxDb clearError after short frame hasError=");
  Serial.print(rxDb.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(rxDb.lastError()));
  Serial.print(" error=");
  Serial.println(rxDb.errorText());

  Serial.print("classicRpm short direct decode ok=");
  Serial.print(classicRpm.decode(rx) ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(classicRpm.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(classicRpm.lastError()));
  Serial.print(" error=");
  Serial.println(classicRpm.errorText());

  classicRpm.clearError();
  Serial.print("classicRpm clearError hasError=");
  Serial.print(classicRpm.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(classicRpm.lastError()));
  Serial.print(" error=");
  Serial.println(classicRpm.errorText());
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(50);
  }

  pinMode(PIN_CAN0_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN0_STANDBY, LOW);

  Serial.println("CANMessageSignal 1.0.0 test");

  ACANFD_SAME_Settings settings(ACANFD_SAME_Settings::CLOCK_48MHz, 500 * 1000, DataBitRateFactor::x1);
  settings.mModuleMode = ACANFD_SAME_Settings::INTERNAL_LOOP_BACK;

  const uint32_t beginStatus = can0.beginFD(settings);
  Serial.print("can0 begin ok=");
  Serial.print(beginStatus == 0 ? "yes" : "no");
  Serial.print(" status=0x");
  Serial.println(beginStatus, HEX);

  rpm.setSignalValue(1000);

  Serial.print("add rpm to classic ok=");
  Serial.print(classicRpm.addSignal(rpm) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(classicRpm.errorText());

  Serial.print("add rpm to FD DLC9 ok=");
  Serial.print(fdDlc9.addSignal(rpm) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(fdDlc9.errorText());

  Serial.print("add constant at byte 11 to FD DLC9 ok=");
  Serial.print(fdDlc9.addConstant(88, 8, 0x55) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(fdDlc9.errorText());

  Serial.print("add rpm to FD auto+BRS ok=");
  Serial.print(fdAutoBrs8.addSignal(rpm) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(fdAutoBrs8.errorText());

  Serial.print("add rpm to invalid classic+BRS ok=");
  Serial.print(invalidClassicBrs.addSignal(rpm) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(invalidClassicBrs.errorText());

  rawECTBig.setSignalValue(80);
  rawECTLittle.setSignalValue(80);
  nibbleBig.setSignalValue(10);
  nibbleLittle.setSignalValue(10);

  Serial.print("add rawECTBig ok=");
  Serial.print(endian8BigTest.addSignal(rawECTBig) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(endian8BigTest.errorText());

  Serial.print("add rawECTLittle ok=");
  Serial.print(endian8LittleTest.addSignal(rawECTLittle) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(endian8LittleTest.errorText());

  Serial.print("add nibbleBig ok=");
  Serial.print(endian4BigTest.addSignal(nibbleBig) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(endian4BigTest.errorText());

  Serial.print("add nibbleLittle ok=");
  Serial.print(endian4LittleTest.addSignal(nibbleLittle) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(endian4LittleTest.errorText());

  int64_t enumValue = -1;
  const bool enumLookupOk = oilGaugeMap.valueForLabel("Fault", enumValue);
  Serial.print("enum lookup Fault ok=");
  Serial.print(enumLookupOk ? "yes" : "no");
  Serial.print(" value=");
  Serial.println((long)enumValue);
  Serial.print("enum label for 2=");
  Serial.println(oilGaugeMap.labelFor(2));

  const bool enumSetOk = oilGaugeEnum.setSignalValue("Fault");
  Serial.print("enum set Fault ok=");
  Serial.print(enumSetOk ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(oilGaugeEnum.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(oilGaugeEnum.lastError()));
  Serial.print(" error=");
  Serial.println(oilGaugeEnum.errorText());

  const bool enumSetBad = oilGaugeEnum.setSignalValue("MissingLabel");
  Serial.print("enum set MissingLabel ok=");
  Serial.print(enumSetBad ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(oilGaugeEnum.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(oilGaugeEnum.lastError()));
  Serial.print(" error=");
  Serial.println(oilGaugeEnum.errorText());

  oilGaugeEnum.clearError();
  Serial.print("enum clearError hasError=");
  Serial.print(oilGaugeEnum.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(oilGaugeEnum.lastError()));
  Serial.print(" error=");
  Serial.println(oilGaugeEnum.errorText());

  const bool enumSetNoMap = noEnumSignal.setSignalValue("Fault");
  Serial.print("enum set on non-enum signal ok=");
  Serial.print(enumSetNoMap ? "yes" : "no");
  Serial.print(" hasError=");
  Serial.print(noEnumSignal.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(noEnumSignal.lastError()));
  Serial.print(" error=");
  Serial.println(noEnumSignal.errorText());
  noEnumSignal.clearError();

  // Set the enum signal back to a valid value before registering/packing it.
  oilGaugeEnum.setSignalValue("Fault");

  Serial.print("add oilGaugeEnum ok=");
  Serial.print(enumLabelTest.addSignal(oilGaugeEnum) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(enumLabelTest.errorText());

  printPacked("classic", classicRpm);
  printPacked("fdDlc9", fdDlc9);
  printPacked("fdAutoBrs8", fdAutoBrs8);
  printPacked("endian8Big rawECT 80C expect byte3=78", endian8BigTest);
  printPacked("endian8Little rawECT 80C expect byte3=78", endian8LittleTest);
  printPacked("endian4Big value A expect byte2=A0", endian4BigTest);
  printPacked("endian4Little value A expect byte2=0A", endian4LittleTest);

  byteBig16.setSignalValue(0x1234);
  byteLittle16.setSignalValue(0x1234);
  bitAbsolute.setSignalValue(true);
  bitByte.setSignalValue(true);
  activeLowBit.setSignalValue(true);
  enumBitSignal.setSignalValue("On");

  Serial.print("add byteBig16 ok=");
  Serial.print(byteBigTest.addSignal(byteBig16) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(byteBigTest.errorText());

  Serial.print("add byteLittle16 ok=");
  Serial.print(byteLittleTest.addSignal(byteLittle16) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(byteLittleTest.errorText());

  Serial.print("add bitAbsolute ok=");
  Serial.print(bitAbsoluteTest.addSignal(bitAbsolute) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(bitAbsoluteTest.errorText());

  Serial.print("add bitByte ok=");
  Serial.print(bitByteTest.addSignal(bitByte) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(bitByteTest.errorText());

  Serial.print("add activeLowBit ok=");
  Serial.print(activeLowBitTest.addSignal(activeLowBit) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(activeLowBitTest.errorText());

  Serial.print("add enumBitSignal ok=");
  Serial.print(enumBitTest.addSignal(enumBitSignal) ? "yes" : "no");
  Serial.print(" error=");
  Serial.println(enumBitTest.errorText());

  printPacked("enum oilGauge Fault expect byte1=02", enumLabelTest);
  printPacked("byteBig16 0x1234 expect 12 34", byteBigTest);
  printPacked("byteLittle16 0x1234 expect 34 12", byteLittleTest);
  printPacked("bitAbsolute true expect byte4=80", bitAbsoluteTest);
  printPacked("bitByte true expect byte2=01", bitByteTest);
  printPacked("activeLowBit true expect byte1=EF", activeLowBitTest);
  printPacked("enumBitSignal On expect byte0=20", enumBitTest);

  microCan0.setBusType(CAN_FD);
  Serial.println("bus type CAN_FD");

  sendAndPrint("classic", classicRpm);
  printReceived("classic loopback");
  printSendIfDueState("classic immediate not due", microCan0.sendIfDue(classicRpm, 1000));

  sendAndPrint("fdDlc9", fdDlc9);
  printReceived("fdDlc9 loopback");

  sendAndPrint("fdAutoBrs8", fdAutoBrs8);
  printReceived("fdAutoBrs8 loopback");

  sendAndPrint("invalid classic+BRS", invalidClassicBrs);
  microCan0.clearError();
  Serial.print("channel clearError after invalid classic+BRS hasError=");
  Serial.print(microCan0.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(microCan0.lastError()));
  Serial.print(" error=");
  Serial.println(microCan0.errorText());

  microCan0.setBusType(CAN_CLASSIC);
  Serial.println("bus type CAN_CLASSIC");

  sendAndPrint("classic on classic bus", classicRpm);
  printReceived("classic on classic bus loopback");

  sendAndPrint("fdDlc9 blocked on classic bus", fdDlc9);
  microCan0.clearError();
  Serial.print("channel clearError after blocked FD hasError=");
  Serial.print(microCan0.hasError() ? "yes" : "no");
  Serial.print(" lastError=");
  Serial.print(int(microCan0.lastError()));
  Serial.print(" error=");
  Serial.println(microCan0.errorText());

  sendAndPrint("fdAutoBrs8 blocked on classic bus", fdAutoBrs8);

  runDecodeTests();

  Serial.println("CANMessageSignal 1.0.0 test complete");
}

void loop() {
}
