#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include "EnumMap.h"

namespace CANMessageSignal {

enum SignalDataType : uint8_t {
  UNSIGNED = 0,
  SIGNED,
  FLOAT,
  DOUBLE
};

// Do not define enum values named LITTLE_ENDIAN or BIG_ENDIAN here.
// Some Arduino/newlib cores already define those names as macros in endian.h.
// The API deliberately accepts those existing constants in .endianness.
// Endianness controls DBC start-bit interpretation for all bit lengths.
// BIG_ENDIAN: startBit is the MSb and bits walk downward with wrap.
// LITTLE_ENDIAN: startBit is the LSb and bits walk upward.
typedef uint16_t SignalEndianness;

// DBC-style multiplexing metadata is included for future API compatibility.
// This release does not implement multiplexed packing, decoding, or
// multiplex-aware overlap validation. Use NORMAL_SIGNAL for supported
// runtime behaviour in this release.
enum SignalRole : uint8_t {
  NORMAL_SIGNAL = 0,
  MULTIPLEXOR,
  MULTIPLEXED_SIGNAL
};

enum SignalOwner : uint8_t {
  SIGNAL_OWNER_SKETCH = 0,
  SIGNAL_OWNER_WEBAPP_OVERRIDE
};

// Intentionally kept as a plain aggregate with no constructors and no default
// member initializers. This is required for the GNU++11 designated-initializer
// syntax used by the public API, for example:
//   CanSignal rpm({ .name = "RPM", .dataType = UNSIGNED, ... });
// Omitted fields are zero/null initialized by the compiler.
struct CanSignalConfig {
  const char* name;
  SignalDataType dataType;
  uint16_t startBit;
  uint16_t bitLength;
  SignalEndianness endianness;
  double factor;
  double offset;
  const char* unit;
  const char* comment;
  double min;
  double max;
  double defaultValue;
  // Multiplexing fields are reserved metadata in this release.
  // They are stored and validated, but do not change pack/decode behaviour.
  SignalRole signalRole;
  const char* multiplexor;
  int64_t multiplexValue;
  const EnumMap* enumMap;
  bool overrideCapable;
};

class CanSignal {
  friend class CanMessage;

public:
  enum ErrorCode : uint8_t {
    OK = 0,
    NULL_NAME,
    INVALID_BIT_LENGTH,
    INVALID_START_BIT,
    INVALID_FACTOR,
    FLOAT_REQUIRES_32_BITS,
    DOUBLE_REQUIRES_64_BITS,
    INVALID_RANGE,
    INVALID_ENUM_MAP,
    ENUM_VALUE_OUT_OF_RANGE,
    MULTIPLEXED_SIGNAL_REQUIRES_MULTIPLEXOR,
    NO_ENUM_MAP,
    INVALID_ENUM_LABEL
  };

  CanSignal() = default;
  explicit CanSignal(const CanSignalConfig& config);

  bool configure(const CanSignalConfig& config);
  bool isValid() const { return m_configValid; }
  ErrorCode errorCode() const { return m_error; }
  ErrorCode lastError() const { return m_error; }
  bool hasError() const { return m_error != OK; }
  void clearError();
  const char* errorText() const;

  const CanSignalConfig& config() const { return m_config; }

  const char* name() const { return m_config.name; }
  SignalDataType dataType() const { return m_config.dataType; }
  uint16_t startBit() const { return m_config.startBit; }
  uint16_t bitLength() const { return m_config.bitLength; }
  SignalEndianness endianness() const { return m_config.endianness; }
  const EnumMap* enumMap() const { return m_config.enumMap; }
  bool isEnum() const { return m_config.enumMap != nullptr; }
  bool overrideCapable() const { return m_config.overrideCapable; }

  void setSignalValue(double value);
  void setSignalValue(int value) { setSignalValue(double(value)); }
  void setSignalValue(unsigned int value) { setSignalValue(double(value)); }
  void setSignalValue(long value) { setSignalValue(double(value)); }
  void setSignalValue(unsigned long value) { setSignalValue(double(value)); }
  void setSignalValue(long long value) { setSignalValue(double(value)); }
  void setSignalValue(unsigned long long value) { setSignalValue(double(value)); }
  void setSignalValue(bool value) { setSignalValue(value ? 1.0 : 0.0); }
  bool setSignalValue(const char* enumLabel);
  double sketchSignalValue() const { return m_sketchValue; }

  // Current physical signal value. For transmit this is the sketch value unless
  // a webapp override is active. After decode(), this is the decoded physical value.
  double signalValue() const { return effectiveSignalValue(); }

  // Last raw integer value decoded from a received frame, or calculated when an
  // enum label is set. For FLOAT/DOUBLE this is the raw bit pattern cast to int64_t.
  int64_t rawSignalValue() const { return m_rawSignalValue; }

  bool hasReceivedValue() const { return m_hasReceivedValue; }
  uint32_t lastUpdateMs() const { return m_lastUpdateMs; }
  const char* enumLabel() const;

  bool setWebappOverrideValue(double value);
  void releaseWebappOverride();
  bool overrideActive() const { return m_overrideActive; }
  double webappOverrideValue() const { return m_webappValue; }

  double effectiveSignalValue() const;
  SignalOwner owner() const;

  void resetToDefault();

  bool rawIntegerRange(int64_t& minRaw, uint64_t& maxRaw) const;
  bool physicalToRawInteger(double physicalValue, int64_t& rawOut) const;

private:
  CanSignalConfig m_config;
  ErrorCode m_error = OK;
  bool m_configValid = false;
  double m_sketchValue = 0.0;
  double m_webappValue = 0.0;
  bool m_overrideActive = false;
  int64_t m_rawSignalValue = 0;
  bool m_hasReceivedValue = false;
  uint32_t m_lastUpdateMs = 0;

  bool updateFromDecodedRaw(uint64_t rawValue, uint32_t nowMs);
  static int64_t signExtend(uint64_t rawValue, uint16_t bitLength);

  ErrorCode validateConfig(const CanSignalConfig& config) const;
  bool enumValuesFit(const CanSignalConfig& config) const;
  bool valueFitsRawRange(int64_t rawValue, const CanSignalConfig& config) const;
};


// Convenience byte-aligned signal definition. This is converted into a normal
// CanSignal internally, so the existing packing and validation path is reused.
// startByte is the first byte occupied by the signal. For BIG_ENDIAN it is the
// most-significant byte. For LITTLE_ENDIAN it is the least-significant byte.
// byteLength is 1..8 and becomes bitLength = byteLength * 8.
struct CanByteSignalConfig {
  const char* name;
  SignalDataType dataType;
  uint8_t startByte;
  uint8_t byteLength;
  SignalEndianness endianness;
  double factor;
  double offset;
  const char* unit;
  const char* comment;
  double min;
  double max;
  double defaultValue;
  bool overrideCapable;
};

class CanByteSignal : public CanSignal {
public:
  CanByteSignal() = default;
  explicit CanByteSignal(const CanByteSignalConfig& config);

protected:
  static CanSignalConfig toSignalConfig(const CanByteSignalConfig& config);
};

// Convenience single-bit signal using an absolute payload bit number.
// bit is 0-based. For a 1-bit signal, endianness is not exposed because there
// is no bit walk beyond the selected bit.
// defaultValue and setSignalValue(0/1/false/true) are logical values. If
// inverted is true, logical 1/true packs raw 0, and logical 0/false packs raw 1.
struct CanBitSignalConfig {
  const char* name;
  uint16_t bit;
  const char* comment;
  bool defaultValue;
  // Multiplexing fields are reserved metadata in this release.
  // They are stored and validated, but do not change pack/decode behaviour.
  SignalRole signalRole;
  const char* multiplexor;
  int64_t multiplexValue;
  const EnumMap* enumMap;
  bool inverted;
  bool overrideCapable;
};

class CanBitSignal : public CanSignal {
public:
  CanBitSignal() = default;
  explicit CanBitSignal(const CanBitSignalConfig& config);

protected:
  static CanSignalConfig toSignalConfig(const CanBitSignalConfig& config);
};

// Convenience single-bit signal using byte + bit-within-byte coordinates.
// byte is 0-based. bit is 0..7 where bit 0 is the LSB and bit 7 is the MSB
// of that byte. Internally this becomes absolute bit = byte * 8 + bit.
struct CanByteBitSignalConfig {
  const char* name;
  uint8_t byte;
  uint8_t bit;
  const char* comment;
  bool defaultValue;
  // Multiplexing fields are reserved metadata in this release.
  // They are stored and validated, but do not change pack/decode behaviour.
  SignalRole signalRole;
  const char* multiplexor;
  int64_t multiplexValue;
  const EnumMap* enumMap;
  bool inverted;
  bool overrideCapable;
};

class CanByteBitSignal : public CanSignal {
public:
  CanByteBitSignal() = default;
  explicit CanByteBitSignal(const CanByteBitSignalConfig& config);

protected:
  static CanSignalConfig toSignalConfig(const CanByteBitSignalConfig& config);
};

} // namespace CANMessageSignal
