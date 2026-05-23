#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include "CanSignal.h"
#include <ACANFD_SAME-from-cpp.h>

#ifndef CANMS_MESSAGE_MAX_SIGNALS
#define CANMS_MESSAGE_MAX_SIGNALS 32
#endif

#ifndef CANMS_MESSAGE_MAX_CONSTANTS
#define CANMS_MESSAGE_MAX_CONSTANTS 32
#endif

namespace CANMessageSignal {

enum CanIdType : uint8_t {
  STANDARD = 0,
  EXTENDED = 1
};

// FRAME_AUTO preserves the simple API: dlc <= 8 sends as classic CAN unless
// bitrateSwitch is true. BRS is a CAN FD feature, so FRAME_AUTO + BRS sends
// as CAN FD even when dlc <= 8. dlc > 8 always sends as CAN FD. Explicit
// modes are available when the user wants to force classic CAN or CAN FD.
enum CanFrameFormat : uint8_t {
  FRAME_AUTO = 0,
  FRAME_CLASSIC = 1,
  FRAME_FD = 2
};

// Intentionally kept as a plain aggregate with no constructors and no default
// member initializers. This keeps the public GNU++11 designated-initializer API working.
struct CanMessageConfig {
  const char* name;
  CanIdType idType;
  uint32_t id;
  uint8_t dlc;
  uint8_t defaultFill;
  const char* comment;
  CanFrameFormat frameFormat; // Optional. Omitted/zero means FRAME_AUTO.
  bool bitrateSwitch;         // Optional. true implies CAN FD when frameFormat is FRAME_AUTO.
};

// For constants, endianness is optional when bitLength <= 8.
// If omitted for bitLength <= 8, LITTLE_ENDIAN/upward bit walking is used.
// If BIG_ENDIAN is explicitly supplied, DBC Motorola start-bit handling is honoured.
// When bitLength > 8, endianness must be explicitly set to LITTLE_ENDIAN or BIG_ENDIAN.
struct CanConstantConfig {
  uint16_t startBit;
  uint16_t bitLength;
  uint64_t value;
  SignalEndianness endianness;
  const char* comment;
};

class CanMessage {
public:
  enum ErrorCode : uint8_t {
    OK = 0,
    NULL_NAME,
    INVALID_ID_TYPE,
    INVALID_STANDARD_ID,
    INVALID_EXTENDED_ID,
    INVALID_DLC,
    TOO_MANY_SIGNALS,
    TOO_MANY_CONSTANTS,
    INVALID_SIGNAL,
    INVALID_FIELD_RANGE,
    INVALID_ENDIANNESS,
    ENDIANNESS_REQUIRED,
    FIELD_OVERLAP,
    DUPLICATE_SIGNAL_NAME,
    BUFFER_TOO_SMALL,
    VALUE_OUT_OF_RANGE,
    FRAME_ID_MISMATCH,
    FRAME_TYPE_MISMATCH,
    FRAME_TOO_SHORT,
    DECODE_FAILED
  };

  CanMessage() = default;
  explicit CanMessage(const CanMessageConfig& config);

  bool configure(const CanMessageConfig& config);
  bool isValid() const { return m_configValid; }
  ErrorCode errorCode() const { return m_error; }
  ErrorCode lastError() const { return m_error; }
  bool hasError() const { return m_error != OK; }
  void clearError();
  const char* errorText() const;

  const CanMessageConfig& config() const { return m_config; }
  const char* name() const { return m_config.name; }
  CanIdType idType() const { return m_config.idType; }
  uint32_t id() const { return m_config.id; }
  // DBC/webapp-compatible raw CAN DLC code: 0..15.
  uint8_t dlc() const { return m_config.dlc; }

  // Actual number of payload bytes represented by dlc().
  // For CAN/CAN FD this maps 0..8 directly, then 9->12, 10->16,
  // 11->20, 12->24, 13->32, 14->48, 15->64.
  uint8_t lengthBytes() const { return dlcToLength(m_config.dlc); }

  uint8_t defaultFill() const { return m_config.defaultFill; }
  const char* comment() const { return m_config.comment; }
  CanFrameFormat frameFormat() const { return m_config.frameFormat; }
  bool bitrateSwitch() const { return m_config.bitrateSwitch; }

  bool classicFrameAllowed() const;
  bool fdFrameAllowed() const;
  bool validTransportLength() const;
  bool requiresCanFd() const;

  uint32_t lastSentMs() const { return m_lastSentMs; }
  bool hasBeenSent() const { return m_hasBeenSent; }
  void markSent(uint32_t nowMs) { m_lastSentMs = nowMs; m_hasBeenSent = true; }

  bool addSignal(CanSignal& signal);

  bool addConstant(uint16_t startBit, uint16_t bitLength, uint64_t value);
  bool addConstant(uint16_t startBit, uint16_t bitLength, uint64_t value, SignalEndianness endianness);
  bool addConstant(uint16_t startBit, uint16_t bitLength, uint64_t value, SignalEndianness endianness, const char* comment);
  bool addConstant(const CanConstantConfig& constantConfig);

  uint8_t signalCount() const { return m_signalCount; }
  const CanSignal* signalAt(uint8_t index) const;
  CanSignal* signalAt(uint8_t index);

  uint8_t constantCount() const { return m_constantCount; }
  bool constantAt(uint8_t index, CanConstantConfig& out) const;

  uint16_t payloadBitLength() const { return uint16_t(lengthBytes()) * 8U; }

  // Packs the current effective signal values and constants into outPayload.
  // outSize must be at least lengthBytes(). Returns false and sets errorCode() on failure.
  bool pack(uint8_t* outPayload, uint8_t outSize);

  // Receive/decode helpers. decode() returns false without setting an error
  // when the frame is simply not this message. If the frame header matches but
  // the payload is invalid, decode() returns false and sets an error.
  bool matches(const CANMessage& frame) const;
  bool matches(const CANFDMessage& frame) const;
  bool decode(const CANMessage& frame);
  bool decode(const CANFDMessage& frame);
  bool decodePayload(const uint8_t* data, uint8_t lengthBytes);

private:
  CanMessageConfig m_config;
  ErrorCode m_error = OK;
  bool m_configValid = false;

  CanSignal* m_signals[CANMS_MESSAGE_MAX_SIGNALS] = {};
  uint8_t m_signalCount = 0;

  CanConstantConfig m_constants[CANMS_MESSAGE_MAX_CONSTANTS] = {};
  uint8_t m_constantCount = 0;

  // Occupancy bitset for max CAN FD payload: 64 bytes = 512 bits.
  uint64_t m_occupiedBits[8] = {};
  uint32_t m_lastSentMs = 0;
  bool m_hasBeenSent = false;

  ErrorCode validateConfig(const CanMessageConfig& config) const;
  ErrorCode validateSignalForMessage(const CanSignal& signal) const;
  ErrorCode validateConstantForMessage(const CanConstantConfig& constantConfig) const;

  bool signalNameExists(const char* name) const;
  bool fieldFitsMessage(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness) const;
  bool fieldOverlaps(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness) const;
  void markFieldOccupied(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness);

  bool enumerateFieldBits(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness, uint16_t* outBits, uint16_t maxOut) const;
  bool fieldBitSequenceValid(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness) const;
  bool constantValueFits(const CanConstantConfig& constantConfig) const;

  bool packRawValue(uint8_t* outPayload, uint16_t startBit, uint16_t bitLength, SignalEndianness endianness, uint64_t rawValue) const;
  bool extractRawValue(const uint8_t* payload, uint16_t startBit, uint16_t bitLength, SignalEndianness endianness, uint64_t& outRawValue) const;
  bool decodeSignal(const uint8_t* payload, CanSignal& signal) const;
  bool packConstant(uint8_t* outPayload, const CanConstantConfig& constantConfig) const;
  bool packSignal(uint8_t* outPayload, const CanSignal& signal) const;
  static uint64_t maskForBitLength(uint16_t bitLength);
  static bool doubleToFloatBits(double value, uint32_t& outBits);
  static uint64_t doubleToDoubleBits(double value);
  static void writePayloadBit(uint8_t* outPayload, uint16_t payloadBit, bool value);
  static bool readPayloadBit(const uint8_t* payload, uint16_t payloadBit);
  static bool isExplicitEndianness(SignalEndianness endianness);
  static uint16_t nextBigEndianBit(uint16_t bit);
  static uint8_t dlcToLength(uint8_t dlc);
  static bool isValidDlcCode(uint8_t dlc);
  bool isBitOccupied(uint16_t bit) const;
  void setBitOccupied(uint16_t bit);
  void clearOccupancy();
};

} // namespace CANMessageSignal
