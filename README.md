# CANMessageSignal 1.0.0

This library uses the concept of messages and signals to send data over the CAN bus. It largely follows the Vector DBC format, but adds a few convenience classes to make common byte and bit based signals easier to define.

The idea is that you define one or more signals per message. You start by building your CAN bus message, which consists of the ID, friendly name, DLC, default fill value, frame format, and other metadata. You then create one or more signals, which consist of items such as signal name, start bit, bit length, scaling, offset, and unit. Each signal is then attached to a message.

You can attach one signal to one or more messages. This could be useful when data is duplicated across one or more IDs, or when you want to transmit the same signal on different channels.

This library depends on ACANFD_SAME <https://github.com/Terry-Gould/ACANFD_SAME>

It is recommended that you get to know ACANFD_SAME before using this library, read the examples and documentation so you are aware how to set up and use the CAN controllers. 

To use the library in a sketch:

```cpp
#include "ACANFD_SAME.h"
#include "CANMessageSignal.h"

using namespace CANMessageSignal;
```

The namespace is used to avoid clashes with other Arduino, CAN, and system libraries. If you do not want to use `using namespace CANMessageSignal;`, you can write the full names instead, for example `CANMessageSignal::CanSignal`.

## CAN message definition

To define a CAN bus message, use the following structure:

```cpp
CanMessage engineStatus({
  .name = "EngineStatus", // Friendly name used for debug, reporting, and control.
  .idType = STANDARD,
  .id = 0x100,
  .dlc = 8,
  .defaultFill = 0x00,
  .comment = "Engine status message" // Friendly comment. AKA description.
});
```

`.dlc` is the field used when defining the message. It means the raw CAN / DBC DLC code, not necessarily the number of payload bytes. Do not use `.lengthBytes` inside the `CanMessage({...})` definition. `lengthBytes()` is a helper function you can call later if you want to query the actual payload length.

For classic CAN this is simple:

```text
DLC 0..8 = 0..8 payload bytes
```

For CAN FD:

```text
DLC 9  = 12 payload bytes
DLC 10 = 16 payload bytes
DLC 11 = 20 payload bytes
DLC 12 = 24 payload bytes
DLC 13 = 32 payload bytes
DLC 14 = 48 payload bytes
DLC 15 = 64 payload bytes
```

After the message has been created, the library provides both:

```cpp
engineStatus.dlc();          // Returns the raw DLC code, 0..15.
engineStatus.lengthBytes();  // Returns the actual payload length in bytes.
```

For example:

```cpp
CanMessage fdMessage({
  .name = "FDMessage",
  .idType = STANDARD,
  .id = 0x301,
  .dlc = 9,              // Raw DLC code 9 = 12 payload bytes.
  .defaultFill = 0x00,
  .comment = "CAN FD 12 byte message"
});

Serial.println(fdMessage.dlc());          // Prints 9.
Serial.println(fdMessage.lengthBytes());  // Prints 12.
```

## Frame format

The message can optionally specify the frame format:

```cpp
.frameFormat = FRAME_AUTO
.frameFormat = FRAME_CLASSIC
.frameFormat = FRAME_FD
```

If `.frameFormat` is not given, it defaults to `FRAME_AUTO`.

The automatic behaviour is:

```text
FRAME_AUTO + dlc <= 8 + bitrateSwitch false -> classic CAN
FRAME_AUTO + dlc <= 8 + bitrateSwitch true  -> CAN FD with BRS
FRAME_AUTO + dlc >= 9                       -> CAN FD
```

If you need to send a CAN FD frame with DLC 0..8 and no BRS, you must explicitly use:

```cpp
.frameFormat = FRAME_FD,
.bitrateSwitch = false
```

This is needed because DLC 0..8 can be either classic CAN or CAN FD.

## Normal bit-level signal definition

To define a normal DBC-style signal, use `CanSignal`:

```cpp
CanSignal rpm({
  .name = "EngineRPM", // Friendly name used for debug, reporting, and control.
  .dataType = UNSIGNED, // UNSIGNED, SIGNED, FLOAT, or DOUBLE.
  .startBit = 24,
  .bitLength = 16,
  .endianness = LITTLE_ENDIAN, // LITTLE_ENDIAN aka Intel, BIG_ENDIAN aka Motorola.
  .factor = 0.25, // AKA multiplier. Must be non-zero.
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
```

Endianness controls DBC start-bit interpretation for all bit lengths.

For `LITTLE_ENDIAN`:

```text
startBit is the LSb of the signal
bits walk upward
```

For `BIG_ENDIAN`:

```text
startBit is the MSb of the signal
bits walk downward with wrap
```

This matters even for 8-bit and sub-byte signals. For example, an 8-bit big-endian signal at `startBit = 31` occupies byte 3, bits 31..24. An 8-bit little-endian signal at `startBit = 24` also occupies byte 3, bits 24..31.

### Multiplexing status

The signal config includes these DBC-style fields for future compatibility:

```cpp
.signalRole = NORMAL_SIGNAL,
.multiplexor = nullptr,
.multiplexValue = 0,
```

In version 1.0.0 these fields are **metadata only**. Multiplexed packing, multiplexed decoding, and multiplex-aware overlap validation are not implemented yet.

This means:

- `NORMAL_SIGNAL` is the only fully supported runtime signal role.
- `MULTIPLEXOR` and `MULTIPLEXED_SIGNAL` are accepted by the API for future-proofing, but they are not used to select which signals are packed or decoded.
- Multiplexed signals are currently validated like normal signals, so overlapping multiplexed layouts will still be rejected with `FIELD_OVERLAP`.
- Do not rely on multiplexing for runtime behaviour in this release.

If you need multiplexed messages today, define separate `CanMessage` objects or handle the multiplexing logic in your sketch code.

## Byte-level signal definition

For byte-aligned numeric values, you can use `CanByteSignal`.

```cpp
CanByteSignal rpm({
  .name = "EngineRPM",
  .dataType = UNSIGNED,
  .startByte = 0,
  .byteLength = 2,
  .endianness = BIG_ENDIAN,
  .factor = 0.26,
  .offset = 0.0,
  .unit = "rpm",
  .comment = "Engine speed",
  .min = 0,
  .max = 10000,
  .defaultValue = 0,
  .overrideCapable = true
});
```

`CanByteSignal` is just a convenience wrapper around `CanSignal`. It uses the same packing and validation code.

For `CanByteSignal`:

```text
startByte is the first byte of the byte-aligned signal field.
byteLength is the number of bytes in the signal. Current range: 1..8.
```

For `BIG_ENDIAN`:

```text
startByte is the most significant byte of the value.
The occupied bytes run upward through the payload.
```

For example, `startByte = 0`, `byteLength = 2`, value `0x1234` gives:

```text
Byte0 = 0x12
Byte1 = 0x34
```

For `LITTLE_ENDIAN`:

```text
startByte is the least significant byte of the value.
The occupied bytes run upward through the payload.
```

For example, `startByte = 0`, `byteLength = 2`, value `0x1234` gives:

```text
Byte0 = 0x34
Byte1 = 0x12
```

`CanByteSignal` does not support enum maps or multiplexing in this version. If you need that, use normal `CanSignal`.

## Single-bit signals

For single-bit signals, you can use `CanBitSignal` or `CanByteBitSignal`.

`CanBitSignal` uses an absolute payload bit number:

```cpp
CanBitSignal cel({
  .name = "CheckEngineLight",
  .bit = 39,
  .comment = "Check Engine Light",
  .defaultValue = 0,
  .enumMap = nullptr,
  .inverted = false,
  .overrideCapable = true
});
```

`CanByteBitSignal` uses byte number and bit number within that byte:

```cpp
CanByteBitSignal celFlash({
  .name = "CheckEngineLightFlashing",
  .byte = 7,
  .bit = 0,
  .comment = "Makes the CEL flash, overrides CEL on",
  .defaultValue = 0,
  .enumMap = nullptr,
  .inverted = false,
  .overrideCapable = true
});
```

For `CanByteBitSignal`:

```text
byte = payload byte index, starting at byte 0
bit = bit within that byte, 0..7
bit 0 = LSb
bit 7 = MSb
```

Single-bit signals hide endianness because there is only one bit.

If `.inverted = true`, `setSignalValue()` sets the logical value and the library packs the opposite raw bit:

```cpp
activeLowLamp.setSignalValue(true);  // packs raw 0
activeLowLamp.setSignalValue(1);     // packs raw 0
activeLowLamp.setSignalValue(false); // packs raw 1
activeLowLamp.setSignalValue(0);     // packs raw 1
```

If `.inverted = false`, the logical value is packed directly.

## Enum maps

To define an `EnumMap`:

```cpp
EnumMap gearMap({
  {0, "Park"},
  {1, "Reverse"},
  {2, "Neutral"},
  {3, "Drive"}
});
```

Some ARM GCC versions may print a note about std::initializer_list parameter passing changing in GCC 7.1. This is a compiler ABI note, not a library error. It can be suppressed with -Wno-psabi if desired.

Then attach it to a signal:

```cpp
CanSignal gearSignal({
  .name = "Gear",
  .dataType = UNSIGNED,
  .startBit = 0,
  .bitLength = 8,
  .endianness = LITTLE_ENDIAN,
  .factor = 1,
  .offset = 0,
  .unit = "",
  .comment = "Gear selector position",
  .min = 0,
  .max = 3,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = &gearMap,
  .overrideCapable = true
});
```

Remember that while `.enumMap` is optional, if used, the `EnumMap` must be defined before the signal object.

Enum maps can be used in both directions:

```cpp
const char* label = gearMap.labelFor(3); // "Drive"
```

and:

```cpp
int64_t value = 0;
bool ok = gearMap.valueForLabel("Drive", value); // ok=true, value=3
```

You can also set enum signals by label:

```cpp
gearSignal.setSignalValue("Drive");
oilGauge.setSignalValue("Fault");
```

For enum label setting, the enum value is treated as the raw signal value. Scaling is not applied to the enum label.

## Attaching signals to messages

You can attach signals to messages like this:

```cpp
engineStatus.addSignal(rpm);
engineStatus.addSignal(speedo);
```

The library automatically checks for the following:

- signal bit range fits within the actual payload length represented by `.dlc`
- signal does not overlap another signal in the message
- signal does not overlap a constant field in the message
- signal name is unique within the message
- FLOAT is 32 bits
- DOUBLE is 64 bits
- enum signals have valid value maps
- enum values fit inside the signal bit length

## Constant fields

You can also add constant, persistent values to the message like this:

```cpp
engineStatus.addConstant(56, 8, 0xFF, LITTLE_ENDIAN);
engineStatus.addConstant(56, 8, 0xFF); // Endianness is optional for bitLength <= 8.
```

Or this:

```cpp
engineStatus.addConstant({
  .startBit = 56,
  .bitLength = 8,
  .value = 0xFF,
  .endianness = LITTLE_ENDIAN, // Optional for bitLength <= 8. Required for fields wider than 8 bits.
  .comment = "Reserved/padding"
});
```

This is useful when padding or reserved values differ from the default. This is not strictly needed. You should only use it when you are certain that the value will never need to change. Constants are not controllable over serial/USB.

You could achieve the same thing by defining a signal instead:

```cpp
CanSignal unknown1({
  .name = "UnknownByte1",
  .dataType = UNSIGNED,
  .startBit = 56,
  .bitLength = 8,
  .endianness = LITTLE_ENDIAN,
  .factor = 1,
  .offset = 0,
  .unit = "",
  .comment = "Unknown byte",
  .min = 0,
  .max = 255,
  .defaultValue = 0,
  .signalRole = NORMAL_SIGNAL,
  .multiplexor = nullptr,
  .multiplexValue = 0,
  .enumMap = nullptr,
  .overrideCapable = true
});

engineStatus.addSignal(unknown1);
unknown1.setSignalValue(0xFF);
```

This way, `0xFF` can be changed if needed without having to recompile the firmware.

## Setting signal values

To set a normal signal value:

```cpp
rpm.setSignalValue(1500);
rpm.setSignalValue(readRpm());
```

For normal numeric signals, this is the physical value. The library converts it using:

```text
raw = (physical - offset) / factor
```

For enum signals, you can use the raw enum number:

```cpp
oilGauge.setSignalValue(2);
```

or the enum label:

```cpp
oilGauge.setSignalValue("Fault");
```

For bit signals, numeric and boolean values are both supported:

```cpp
cel.setSignalValue(true);
cel.setSignalValue(1);
cel.setSignalValue(false);
cel.setSignalValue(0);
```

## Sending messages

Create a channel wrapper:

```cpp
CanChannel can0Channel(can0);
```

One-shot send:

```cpp
can0Channel.sendMessage(engineStatus);
```

This sends a message on the bus once and immediately, or more correctly, sends it to the transmit buffer.

You have to handle the timing yourself if you want periodic sending. This is useful for sending messages once based on an event, or when you need tight control over when a message is sent.

Periodic send:

```cpp
can0Channel.sendIfDue(engineStatus, 20);
```

Must be called repeatedly, for example in `loop()`. Sends only when the specified period has elapsed. It is not completely set and forget; you need to ensure that it is called faster than the period.

`sendIfDue()` returns `true` only when a send was actually attempted and accepted by the ACANFD driver.

It returns `false` in two different situations:

1. the message was not due yet; this is normal and clears the channel error state to `OK`
2. the message was due, but packing or driver transmit failed; in this case `can0Channel.hasError()` will be `true`

Use this pattern if you need to distinguish "not due" from a real failure:

```cpp
const bool sent = can0Channel.sendIfDue(engineStatus, 20);

if (!sent && can0Channel.hasError()) {
  Serial.print("sendIfDue failed: ");
  Serial.println(can0Channel.errorText());
} else if (sent) {
  Serial.println("engineStatus accepted for transmit");
}
```

Internally, the due check is based on the message's `lastSentMs()` timestamp, which is updated after `sendMessage()` succeeds.

## Bus type guard

By default, `CanChannel` allows CAN FD:

```cpp
can0Channel.setBusType(CAN_FD);
```

This allows both classic CAN and CAN FD messages.

If you are connected to a classic CAN 2.0 bus, use:

```cpp
can0Channel.setBusType(CAN_CLASSIC);
```

This prevents accidental CAN FD frames from reaching the ACANFD driver. This is useful because a single CAN FD frame sent onto a classic-only bus can get stuck retrying and block later sends.

Classic-only mode rejects:

```text
.dlc = 9..15
.frameFormat = FRAME_FD
.bitrateSwitch = true
```

## Error handling

`CanChannel` supports:

```cpp
can0Channel.hasError();
can0Channel.lastError();
can0Channel.errorText();
can0Channel.lastDriverStatus();
can0Channel.clearError();
```

`CanSignal` supports:

```cpp
signal.hasError();
signal.lastError();
signal.errorText();
signal.clearError();
```

A simple helper can be used like this:

```cpp
void printCanError(const char* context, CanChannel& channel) {
  if (!channel.hasError()) {
    return;
  }

  Serial.print(context);
  Serial.print(" error=");
  Serial.print(channel.errorText());

  const uint32_t driverStatus = channel.lastDriverStatus();
  if (driverStatus != 0) {
    Serial.print(" driver=0x");
    Serial.print(driverStatus, HEX);
  }

  Serial.println();
}
```

Then:

```cpp
can0Channel.sendIfDue(engineStatus, 50);
printCanError("engineStatus", can0Channel);
```

No error prints nothing. A library error prints the error text. A driver error also prints the driver status.

## External signal definition files

For larger sketches, it is recommended to keep messages and signals in a separate file, for example:

```cpp
#include "DeviceSignals.h"
```

The signal file can include the library:

```cpp
#pragma once
#include "CANMessageSignal.h"

using namespace CANMessageSignal;
```

Then define the signals and messages in that file, and attach them in a function:

```cpp
inline bool setDeviceSignals() {
  bool ok = true;

  ok &= engineStatus.addSignal(rpm);
  ok &= engineStatus.addSignal(speedo);

  return ok;
}
```

Then in the main sketch:

```cpp
if (!setDeviceSignals()) {
  Serial.println("Signal registration failed");
}
```

This keeps the main sketch readable when there are hundreds of signals and dozens of messages.


## Receive decoding

Receive-side decoding is supported. The CAN driver setup is still left to the user. This means filters, FIFOs, callbacks, interrupts, and buffering stay under your control.

The simple pattern is:

```cpp
CANFDMessage rx;

while (can0.receiveFD0(rx) || can0.receiveFD1(rx)) {
  rxDb.decode(rx);
}
```

or from a callback:

```cpp
static void callBackForAnyKnownFrame(const CANFDMessage& inMessage) {
  rxDb.decode(inMessage);
}
```

You define the messages and signals in the same way as for transmit, then add the messages to a database:

```cpp
CanMessageDatabase<32> rxDb;

void setup() {
  rxDb.addMessage(engineStatus);
  rxDb.addMessage(absStatus);
}
```

`CanMessageDatabase` is not channel-aware. It matches messages by CAN ID and standard/extended ID type. In dual-channel applications where the same ID may exist on CAN1 and CAN2, use one database per channel, for example `can1RxDb` and `can2RxDb`.

When a received frame is decoded, the attached signals are updated:

```cpp
if (rxDb.decode(rx)) {
  Serial.print("RPM=");
  Serial.println(rpm.signalValue());

  Serial.print("Raw RPM=");
  Serial.println(rpm.rawSignalValue());
}
```

When a frame is decoded, each attached `CanSignal` stores the decoded physical value in that signal object. In other words, a signal object has one runtime value.

For this reason, if the same real-world signal is both transmitted and received, define separate TX and RX `CanSignal` objects unless you intentionally want them to share state. For example, use names such as `rpmTx` and `rpmRx`.

Useful receive helpers:

```cpp
message.matches(rx);          // true if ID, standard/extended type, and length are compatible
message.decode(rx);           // decodes one specific message
rxDb.decode(rx);              // finds the matching message and decodes it

signal.signalValue();         // current physical value
signal.rawSignalValue();      // last decoded raw value
signal.hasReceivedValue();    // true after successful decode
signal.lastUpdateMs();        // millis() when last decoded
signal.enumLabel();           // label for enum-backed signals, if available
```

`CANFDMessage` can still contain a classic CAN 2.0 frame. Using `CANFDMessage` as the receive container does not by itself mean the received frame was CAN FD. It is just the frame type used by the ACANFD library.

`setBusType(CAN_CLASSIC)` is a transmit safety guard. It prevents accidental CAN FD transmit attempts. Classic CAN frames received into a `CANFDMessage` container can still be decoded normally.


If `rxDb.decode(rx)` returns `false` because the frame ID is not in the database, this is treated as a normal "not decoded" result, not an error. This matters on real CAN buses where the database may only contain the messages you care about:

```cpp
if (rxDb.decode(rx)) {
  Serial.println(rxDb.lastDecodedMessage()->name());
}

if (rxDb.hasError()) {
  Serial.println(rxDb.errorText()); // Only real decode/database errors, not unknown IDs.
}
```

A matching message with a bad payload, such as a frame that is too short, is still treated as an error.

## Dependencies

This library expects the ACANFD_SAME / ACANFD_SAME API to be available. It does not configure the CAN peripheral for you. The user remains responsible for ACANFD setup, message RAM size defines, CAN transceiver standby pins, filters, FIFOs, callbacks, and bus mode.

## What this library does not do

This library is not a full CAN driver. It does not replace ACANFD_SAME. It only provides message/signal definition, packing, sending through a CanChannel wrapper, frame matching, and receive decoding.