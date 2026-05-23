#include "CanSignal.h"
#include <math.h>
#include <string.h>

namespace CANMessageSignal {

CanSignal::CanSignal(const CanSignalConfig& config) {
  configure(config);
}

bool CanSignal::configure(const CanSignalConfig& config) {
  m_error = validateConfig(config);
  m_configValid = false;
  if (m_error != OK) {
    return false;
  }

  m_config = config;
  m_sketchValue = config.defaultValue;
  m_webappValue = config.defaultValue;
  m_overrideActive = false;
  m_rawSignalValue = 0;
  m_hasReceivedValue = false;
  m_lastUpdateMs = 0;
  m_configValid = true;
  m_error = OK;
  return true;
}

CanSignal::ErrorCode CanSignal::validateConfig(const CanSignalConfig& config) const {
  if (config.name == nullptr || config.name[0] == '\0') {
    return NULL_NAME;
  }

  if (config.bitLength == 0 || config.bitLength > 64) {
    return INVALID_BIT_LENGTH;
  }

  // Current library target is CAN FD max payload: 64 bytes = 512 bits.
  // Message-level validation will later check against the actual DLC.
  if (config.startBit >= 512) {
    return INVALID_START_BIT;
  }

  if (config.factor == 0.0) {
    return INVALID_FACTOR;
  }

  if (config.dataType == FLOAT && config.bitLength != 32) {
    return FLOAT_REQUIRES_32_BITS;
  }

  if (config.dataType == DOUBLE && config.bitLength != 64) {
    return DOUBLE_REQUIRES_64_BITS;
  }

  if (config.min > config.max) {
    return INVALID_RANGE;
  }

  if (config.signalRole == MULTIPLEXED_SIGNAL && (config.multiplexor == nullptr || config.multiplexor[0] == '\0')) {
    return MULTIPLEXED_SIGNAL_REQUIRES_MULTIPLEXOR;
  }

  if (config.enumMap != nullptr) {
    if (!config.enumMap->isValid()) {
      return INVALID_ENUM_MAP;
    }
    if (!enumValuesFit(config)) {
      return ENUM_VALUE_OUT_OF_RANGE;
    }
  }

  return OK;
}

bool CanSignal::enumValuesFit(const CanSignalConfig& config) const {
  if (config.enumMap == nullptr) return true;

  for (uint8_t i = 0; i < config.enumMap->count(); i++) {
    if (!valueFitsRawRange(config.enumMap->valueAt(i), config)) {
      return false;
    }
  }
  return true;
}

bool CanSignal::valueFitsRawRange(int64_t rawValue, const CanSignalConfig& config) const {
  if (config.dataType == SIGNED) {
    if (config.bitLength >= 64) return true;
    const int64_t minRaw = -(int64_t(1) << (config.bitLength - 1));
    const int64_t maxRaw =  (int64_t(1) << (config.bitLength - 1)) - 1;
    return rawValue >= minRaw && rawValue <= maxRaw;
  }

  // Enums on FLOAT/DOUBLE signals are not expected, but if supplied the raw enum
  // value still needs to fit an unsigned integer representation of bitLength.
  if (rawValue < 0) return false;
  if (config.bitLength >= 64) return true;
  const uint64_t maxRaw = (uint64_t(1) << config.bitLength) - 1ULL;
  return uint64_t(rawValue) <= maxRaw;
}

void CanSignal::setSignalValue(double value) {
  m_sketchValue = value;
}

void CanSignal::clearError() {
  if (m_configValid) {
    m_error = OK;
  }
}

bool CanSignal::setSignalValue(const char* enumLabel) {
  if (!m_configValid) {
    return false;
  }

  if (m_config.enumMap == nullptr) {
    m_error = NO_ENUM_MAP;
    return false;
  }

  if (enumLabel == nullptr) {
    m_error = INVALID_ENUM_LABEL;
    return false;
  }

  int64_t rawValue = 0;
  if (!m_config.enumMap->valueForLabel(enumLabel, rawValue)) {
    m_error = INVALID_ENUM_LABEL;
    return false;
  }

  // EnumMap values are raw CAN signal values. Store the corresponding
  // physical value so the existing pack path converts back to the same raw value.
  m_sketchValue = (double(rawValue) * m_config.factor) + m_config.offset;
  m_rawSignalValue = rawValue;
  m_error = OK;
  return true;
}


const char* CanSignal::enumLabel() const {
  if (m_config.enumMap == nullptr) return nullptr;
  return m_config.enumMap->labelFor(m_rawSignalValue);
}

int64_t CanSignal::signExtend(uint64_t rawValue, uint16_t bitLength) {
  if (bitLength == 0) return 0;
  if (bitLength >= 64U) return int64_t(rawValue);

  const uint64_t signBit = uint64_t(1) << (bitLength - 1U);
  const uint64_t mask = (uint64_t(1) << bitLength) - 1ULL;
  rawValue &= mask;

  if ((rawValue & signBit) == 0U) {
    return int64_t(rawValue);
  }

  const uint64_t extendMask = ~mask;
  return int64_t(rawValue | extendMask);
}

bool CanSignal::updateFromDecodedRaw(uint64_t rawValue, uint32_t nowMs) {
  if (!m_configValid) {
    return false;
  }

  double physicalValue = 0.0;
  int64_t storedRaw = 0;

  if (m_config.dataType == FLOAT) {
    uint32_t bits = uint32_t(rawValue & 0xFFFFFFFFULL);
    float f = 0.0f;
    memcpy(&f, &bits, sizeof(f));
    physicalValue = double(f);
    storedRaw = int64_t(bits);
  } else if (m_config.dataType == DOUBLE) {
    double d = 0.0;
    memcpy(&d, &rawValue, sizeof(d));
    physicalValue = d;
    storedRaw = int64_t(rawValue);
  } else if (m_config.dataType == SIGNED) {
    const int64_t signedRaw = signExtend(rawValue, m_config.bitLength);
    physicalValue = (double(signedRaw) * m_config.factor) + m_config.offset;
    storedRaw = signedRaw;
  } else {
    physicalValue = (double(rawValue) * m_config.factor) + m_config.offset;
    storedRaw = int64_t(rawValue);
  }

  m_sketchValue = physicalValue;
  m_rawSignalValue = storedRaw;
  m_hasReceivedValue = true;
  m_lastUpdateMs = nowMs;
  m_error = OK;
  return true;
}

bool CanSignal::setWebappOverrideValue(double value) {
  if (!m_config.overrideCapable) {
    return false;
  }
  m_webappValue = value;
  m_overrideActive = true;
  return true;
}

void CanSignal::releaseWebappOverride() {
  m_overrideActive = false;
}

double CanSignal::effectiveSignalValue() const {
  return m_overrideActive ? m_webappValue : m_sketchValue;
}

SignalOwner CanSignal::owner() const {
  return m_overrideActive ? SIGNAL_OWNER_WEBAPP_OVERRIDE : SIGNAL_OWNER_SKETCH;
}

void CanSignal::resetToDefault() {
  m_sketchValue = m_config.defaultValue;
  m_webappValue = m_config.defaultValue;
  m_overrideActive = false;
}

bool CanSignal::rawIntegerRange(int64_t& minRaw, uint64_t& maxRaw) const {
  if (m_config.dataType == FLOAT || m_config.dataType == DOUBLE) {
    minRaw = 0;
    maxRaw = 0;
    return false;
  }

  if (m_config.dataType == SIGNED) {
    if (m_config.bitLength >= 64) {
      minRaw = INT64_MIN;
      maxRaw = uint64_t(INT64_MAX);
    } else {
      minRaw = -(int64_t(1) << (m_config.bitLength - 1));
      maxRaw = uint64_t((int64_t(1) << (m_config.bitLength - 1)) - 1);
    }
  } else {
    minRaw = 0;
    maxRaw = (m_config.bitLength >= 64) ? UINT64_MAX : ((uint64_t(1) << m_config.bitLength) - 1ULL);
  }
  return true;
}

bool CanSignal::physicalToRawInteger(double physicalValue, int64_t& rawOut) const {
  if (m_config.dataType == FLOAT || m_config.dataType == DOUBLE || m_config.factor == 0.0) {
    return false;
  }

  const double rawD = (physicalValue - m_config.offset) / m_config.factor;
  const int64_t raw = int64_t((rawD >= 0.0) ? (rawD + 0.5) : (rawD - 0.5));

  int64_t minRaw = 0;
  uint64_t maxRaw = 0;
  if (!rawIntegerRange(minRaw, maxRaw)) return false;
  if (raw < minRaw) return false;
  if (raw >= 0 && uint64_t(raw) > maxRaw) return false;

  rawOut = raw;
  return true;
}


CanByteSignal::CanByteSignal(const CanByteSignalConfig& config)
  : CanSignal(toSignalConfig(config)) {
}

CanSignalConfig CanByteSignal::toSignalConfig(const CanByteSignalConfig& config) {
  const uint16_t startBit = (config.endianness == BIG_ENDIAN)
    ? uint16_t(uint16_t(config.startByte) * 8U + 7U)
    : uint16_t(uint16_t(config.startByte) * 8U);

  CanSignalConfig out = {
    config.name,
    config.dataType,
    startBit,
    uint16_t(uint16_t(config.byteLength) * 8U),
    config.endianness,
    config.factor,
    config.offset,
    config.unit,
    config.comment,
    config.min,
    config.max,
    config.defaultValue,
    NORMAL_SIGNAL,
    nullptr,
    0,
    nullptr,
    config.overrideCapable
  };
  return out;
}

CanBitSignal::CanBitSignal(const CanBitSignalConfig& config)
  : CanSignal(toSignalConfig(config)) {
}

CanSignalConfig CanBitSignal::toSignalConfig(const CanBitSignalConfig& config) {
  // Use factor/offset to make normal CanSignal numeric setting operate on a
  // logical bit value. Non-inverted: raw = logical. Inverted: raw = 1 - logical.
  const double factor = config.inverted ? -1.0 : 1.0;
  const double offset = config.inverted ? 1.0 : 0.0;

  CanSignalConfig out = {
    config.name,
    UNSIGNED,
    config.bit,
    1,
    LITTLE_ENDIAN,
    factor,
    offset,
    "",
    config.comment,
    0.0,
    1.0,
    config.defaultValue ? 1.0 : 0.0,
    config.signalRole,
    config.multiplexor,
    config.multiplexValue,
    config.enumMap,
    config.overrideCapable
  };
  return out;
}

CanByteBitSignal::CanByteBitSignal(const CanByteBitSignalConfig& config)
  : CanSignal(toSignalConfig(config)) {
}

CanSignalConfig CanByteBitSignal::toSignalConfig(const CanByteBitSignalConfig& config) {
  // If bit > 7, deliberately produce an invalid start bit so normal CanSignal
  // validation rejects the configuration.
  const uint16_t absoluteBit = (config.bit <= 7U)
    ? uint16_t(uint16_t(config.byte) * 8U + uint16_t(config.bit))
    : 512U;

  const double factor = config.inverted ? -1.0 : 1.0;
  const double offset = config.inverted ? 1.0 : 0.0;

  CanSignalConfig out = {
    config.name,
    UNSIGNED,
    absoluteBit,
    1,
    LITTLE_ENDIAN,
    factor,
    offset,
    "",
    config.comment,
    0.0,
    1.0,
    config.defaultValue ? 1.0 : 0.0,
    config.signalRole,
    config.multiplexor,
    config.multiplexValue,
    config.enumMap,
    config.overrideCapable
  };
  return out;
}

const char* CanSignal::errorText() const {
  switch (m_error) {
    case OK: return "OK";
    case NULL_NAME: return "Signal name is null or empty";
    case INVALID_BIT_LENGTH: return "Invalid signal bit length";
    case INVALID_START_BIT: return "Invalid signal start bit";
    case INVALID_FACTOR: return "Signal factor must be non-zero";
    case FLOAT_REQUIRES_32_BITS: return "FLOAT signal requires 32 bits";
    case DOUBLE_REQUIRES_64_BITS: return "DOUBLE signal requires 64 bits";
    case INVALID_RANGE: return "Signal minimum is greater than maximum";
    case INVALID_ENUM_MAP: return "Invalid enum map";
    case ENUM_VALUE_OUT_OF_RANGE: return "Enum value out of signal raw range";
    case MULTIPLEXED_SIGNAL_REQUIRES_MULTIPLEXOR: return "Multiplexed signal requires multiplexor name";
    case NO_ENUM_MAP: return "Signal has no enum map";
    case INVALID_ENUM_LABEL: return "Invalid enum label";
    default: return "Unknown signal error";
  }
}

} // namespace CANMessageSignal
