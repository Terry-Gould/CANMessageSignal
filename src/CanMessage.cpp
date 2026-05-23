#include "CanMessage.h"
#include <string.h>

namespace CANMessageSignal {

CanMessage::CanMessage(const CanMessageConfig& config) {
  configure(config);
}

bool CanMessage::configure(const CanMessageConfig& config) {
  m_error = validateConfig(config);
  if (m_error != OK) {
    m_configValid = false;
    return false;
  }

  m_config = config;
  m_signalCount = 0;
  m_constantCount = 0;
  m_lastSentMs = 0;
  m_hasBeenSent = false;
  clearOccupancy();
  m_configValid = true;
  m_error = OK;
  return true;
}

void CanMessage::clearError() {
  if (m_configValid) {
    m_error = OK;
  }
}

CanMessage::ErrorCode CanMessage::validateConfig(const CanMessageConfig& config) const {
  if (config.name == nullptr || config.name[0] == '\0') {
    return NULL_NAME;
  }

  if (config.idType != STANDARD && config.idType != EXTENDED) {
    return INVALID_ID_TYPE;
  }

  if (config.idType == STANDARD && config.id > 0x7FFUL) {
    return INVALID_STANDARD_ID;
  }

  if (config.idType == EXTENDED && config.id > 0x1FFFFFFFUL) {
    return INVALID_EXTENDED_ID;
  }

  if (config.frameFormat != FRAME_AUTO && config.frameFormat != FRAME_CLASSIC && config.frameFormat != FRAME_FD) {
    return INVALID_ID_TYPE;
  }

  if (!isValidDlcCode(config.dlc)) {
    return INVALID_DLC;
  }

  return OK;
}

bool CanMessage::addSignal(CanSignal& signal) {
  if (!m_configValid) return false;
  if (m_signalCount >= CANMS_MESSAGE_MAX_SIGNALS) {
    m_error = TOO_MANY_SIGNALS;
    return false;
  }

  const ErrorCode err = validateSignalForMessage(signal);
  if (err != OK) {
    m_error = err;
    return false;
  }

  m_signals[m_signalCount++] = &signal;
  markFieldOccupied(signal.startBit(), signal.bitLength(), signal.endianness());
  m_error = OK;
  return true;
}

bool CanMessage::addConstant(uint16_t startBit, uint16_t bitLength, uint64_t value) {
  CanConstantConfig cfg = { startBit, bitLength, value, 0, nullptr };
  return addConstant(cfg);
}

bool CanMessage::addConstant(uint16_t startBit, uint16_t bitLength, uint64_t value, SignalEndianness endianness) {
  CanConstantConfig cfg = { startBit, bitLength, value, endianness, nullptr };
  return addConstant(cfg);
}

bool CanMessage::addConstant(uint16_t startBit, uint16_t bitLength, uint64_t value, SignalEndianness endianness, const char* comment) {
  CanConstantConfig cfg = { startBit, bitLength, value, endianness, comment };
  return addConstant(cfg);
}

bool CanMessage::addConstant(const CanConstantConfig& constantConfig) {
  if (!m_configValid) return false;
  if (m_constantCount >= CANMS_MESSAGE_MAX_CONSTANTS) {
    m_error = TOO_MANY_CONSTANTS;
    return false;
  }

  const ErrorCode err = validateConstantForMessage(constantConfig);
  if (err != OK) {
    m_error = err;
    return false;
  }

  m_constants[m_constantCount++] = constantConfig;
  markFieldOccupied(constantConfig.startBit, constantConfig.bitLength, constantConfig.endianness);
  m_error = OK;
  return true;
}

const CanSignal* CanMessage::signalAt(uint8_t index) const {
  return (index < m_signalCount) ? m_signals[index] : nullptr;
}

CanSignal* CanMessage::signalAt(uint8_t index) {
  return (index < m_signalCount) ? m_signals[index] : nullptr;
}

bool CanMessage::constantAt(uint8_t index, CanConstantConfig& out) const {
  if (index >= m_constantCount) return false;
  out = m_constants[index];
  return true;
}

CanMessage::ErrorCode CanMessage::validateSignalForMessage(const CanSignal& signal) const {
  if (!signal.isValid()) {
    return INVALID_SIGNAL;
  }

  if (signalNameExists(signal.name())) {
    return DUPLICATE_SIGNAL_NAME;
  }

  if (!fieldBitSequenceValid(signal.startBit(), signal.bitLength(), signal.endianness())) {
    return INVALID_ENDIANNESS;
  }

  if (!fieldFitsMessage(signal.startBit(), signal.bitLength(), signal.endianness())) {
    return INVALID_FIELD_RANGE;
  }

  if (fieldOverlaps(signal.startBit(), signal.bitLength(), signal.endianness())) {
    return FIELD_OVERLAP;
  }

  return OK;
}

CanMessage::ErrorCode CanMessage::validateConstantForMessage(const CanConstantConfig& constantConfig) const {
  if (constantConfig.bitLength == 0 || constantConfig.bitLength > 64) {
    return INVALID_FIELD_RANGE;
  }

  if (constantConfig.bitLength > 8 && !isExplicitEndianness(constantConfig.endianness)) {
    return ENDIANNESS_REQUIRED;
  }

  if (!constantValueFits(constantConfig)) {
    return VALUE_OUT_OF_RANGE;
  }

  if (!fieldBitSequenceValid(constantConfig.startBit, constantConfig.bitLength, constantConfig.endianness)) {
    return INVALID_ENDIANNESS;
  }

  if (!fieldFitsMessage(constantConfig.startBit, constantConfig.bitLength, constantConfig.endianness)) {
    return INVALID_FIELD_RANGE;
  }

  if (fieldOverlaps(constantConfig.startBit, constantConfig.bitLength, constantConfig.endianness)) {
    return FIELD_OVERLAP;
  }

  return OK;
}


bool CanMessage::pack(uint8_t* outPayload, uint8_t outSize) {
  if (!m_configValid) {
    m_error = INVALID_DLC;
    return false;
  }
  const uint8_t len = lengthBytes();
  if (outPayload == nullptr || outSize < len) {
    m_error = BUFFER_TOO_SMALL;
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    outPayload[i] = m_config.defaultFill;
  }

  for (uint8_t i = 0; i < m_constantCount; i++) {
    if (!packConstant(outPayload, m_constants[i])) {
      m_error = VALUE_OUT_OF_RANGE;
      return false;
    }
  }

  for (uint8_t i = 0; i < m_signalCount; i++) {
    if (m_signals[i] == nullptr || !packSignal(outPayload, *m_signals[i])) {
      m_error = VALUE_OUT_OF_RANGE;
      return false;
    }
  }

  m_error = OK;
  return true;
}

bool CanMessage::packConstant(uint8_t* outPayload, const CanConstantConfig& constantConfig) const {
  return packRawValue(outPayload,
                      constantConfig.startBit,
                      constantConfig.bitLength,
                      constantConfig.endianness,
                      constantConfig.value);
}

bool CanMessage::packSignal(uint8_t* outPayload, const CanSignal& signal) const {
  const uint16_t bitLength = signal.bitLength();
  uint64_t raw = 0;

  if (signal.dataType() == FLOAT) {
    uint32_t bits = 0;
    if (!doubleToFloatBits(signal.effectiveSignalValue(), bits)) return false;
    raw = uint64_t(bits);
  } else if (signal.dataType() == DOUBLE) {
    raw = doubleToDoubleBits(signal.effectiveSignalValue());
  } else {
    int64_t rawSigned = 0;
    if (!signal.physicalToRawInteger(signal.effectiveSignalValue(), rawSigned)) return false;
    raw = uint64_t(rawSigned) & maskForBitLength(bitLength);
  }

  return packRawValue(outPayload,
                      signal.startBit(),
                      signal.bitLength(),
                      signal.endianness(),
                      raw);
}

bool CanMessage::packRawValue(uint8_t* outPayload, uint16_t startBit, uint16_t bitLength, SignalEndianness endianness, uint64_t rawValue) const {
  uint16_t bits[64];
  if (!enumerateFieldBits(startBit, bitLength, endianness, bits, 64)) return false;

  if (endianness == BIG_ENDIAN) {
    for (uint16_t i = 0; i < bitLength; i++) {
      const uint16_t rawBitIndex = uint16_t(bitLength - 1U - i);
      const bool bitValue = ((rawValue >> rawBitIndex) & 0x1ULL) != 0;
      writePayloadBit(outPayload, bits[i], bitValue);
    }
    return true;
  }

  if (endianness == LITTLE_ENDIAN || bitLength <= 8U) {
    for (uint16_t i = 0; i < bitLength; i++) {
      const bool bitValue = ((rawValue >> i) & 0x1ULL) != 0;
      writePayloadBit(outPayload, bits[i], bitValue);
    }
    return true;
  }

  return false;
}


bool CanMessage::matches(const CANMessage& frame) const {
  if (!m_configValid) return false;
  if (frame.id != m_config.id) return false;
  if (frame.ext != (m_config.idType == EXTENDED)) return false;
  if (frame.rtr) return false;
  if (frame.len < lengthBytes()) return false;
  return true;
}

bool CanMessage::matches(const CANFDMessage& frame) const {
  if (!m_configValid) return false;
  if (frame.id != m_config.id) return false;
  if (frame.ext != (m_config.idType == EXTENDED)) return false;
  if (frame.type == CANFDMessage::CAN_REMOTE) return false;
  if (frame.len < lengthBytes()) return false;
  return true;
}

bool CanMessage::decode(const CANMessage& frame) {
  if (!m_configValid) {
    m_error = INVALID_DLC;
    return false;
  }

  // A frame with a different ID/type is simply "not this message".
  // This allows users to call message.decode(rx) in a generic receive loop
  // without treating every unrelated bus frame as an error.
  if (frame.id != m_config.id ||
      frame.ext != (m_config.idType == EXTENDED) ||
      frame.rtr) {
    m_error = OK;
    return false;
  }

  // Same message header but too-short payload is a real decode error.
  if (frame.len < lengthBytes()) {
    m_error = FRAME_TOO_SHORT;
    return false;
  }

  return decodePayload(frame.data, frame.len);
}

bool CanMessage::decode(const CANFDMessage& frame) {
  if (!m_configValid) {
    m_error = INVALID_DLC;
    return false;
  }

  // A frame with a different ID/type is simply "not this message".
  // CANFDMessage can also carry classic CAN data frames, so the container type
  // alone does not imply that the received frame is CAN FD.
  if (frame.id != m_config.id ||
      frame.ext != (m_config.idType == EXTENDED) ||
      frame.type == CANFDMessage::CAN_REMOTE) {
    m_error = OK;
    return false;
  }

  // Same message header but too-short payload is a real decode error.
  if (frame.len < lengthBytes()) {
    m_error = FRAME_TOO_SHORT;
    return false;
  }

  return decodePayload(frame.data, frame.len);
}

bool CanMessage::decodePayload(const uint8_t* data, uint8_t receivedLengthBytes) {
  if (!m_configValid) {
    m_error = INVALID_DLC;
    return false;
  }

  if (data == nullptr) {
    m_error = BUFFER_TOO_SMALL;
    return false;
  }

  if (receivedLengthBytes < lengthBytes()) {
    m_error = FRAME_TOO_SHORT;
    return false;
  }

  for (uint8_t i = 0; i < m_signalCount; i++) {
    if (m_signals[i] == nullptr || !decodeSignal(data, *m_signals[i])) {
      m_error = DECODE_FAILED;
      return false;
    }
  }

  m_error = OK;
  return true;
}

bool CanMessage::decodeSignal(const uint8_t* payload, CanSignal& signal) const {
  uint64_t raw = 0;
  if (!extractRawValue(payload, signal.startBit(), signal.bitLength(), signal.endianness(), raw)) {
    return false;
  }

  return signal.updateFromDecodedRaw(raw, millis());
}

bool CanMessage::extractRawValue(const uint8_t* payload, uint16_t startBit, uint16_t bitLength, SignalEndianness endianness, uint64_t& outRawValue) const {
  uint16_t bits[64];
  if (!payload || !enumerateFieldBits(startBit, bitLength, endianness, bits, 64)) return false;

  uint64_t raw = 0;

  if (endianness == BIG_ENDIAN) {
    for (uint16_t i = 0; i < bitLength; i++) {
      const uint16_t rawBitIndex = uint16_t(bitLength - 1U - i);
      if (readPayloadBit(payload, bits[i])) {
        raw |= (uint64_t(1) << rawBitIndex);
      }
    }
    outRawValue = raw;
    return true;
  }

  if (endianness == LITTLE_ENDIAN || bitLength <= 8U) {
    for (uint16_t i = 0; i < bitLength; i++) {
      if (readPayloadBit(payload, bits[i])) {
        raw |= (uint64_t(1) << i);
      }
    }
    outRawValue = raw;
    return true;
  }

  return false;
}

void CanMessage::writePayloadBit(uint8_t* outPayload, uint16_t payloadBit, bool value) {
  const uint16_t byteIndex = uint16_t(payloadBit / 8U);
  const uint8_t bitIndex = uint8_t(payloadBit % 8U);
  const uint8_t mask = uint8_t(1U << bitIndex);
  if (value) outPayload[byteIndex] = uint8_t(outPayload[byteIndex] | mask);
  else outPayload[byteIndex] = uint8_t(outPayload[byteIndex] & uint8_t(~mask));
}

bool CanMessage::readPayloadBit(const uint8_t* payload, uint16_t payloadBit) {
  const uint16_t byteIndex = uint16_t(payloadBit / 8U);
  const uint8_t bitIndex = uint8_t(payloadBit % 8U);
  return (payload[byteIndex] & uint8_t(1U << bitIndex)) != 0U;
}

uint64_t CanMessage::maskForBitLength(uint16_t bitLength) {
  return (bitLength >= 64U) ? UINT64_MAX : ((uint64_t(1) << bitLength) - 1ULL);
}

bool CanMessage::doubleToFloatBits(double value, uint32_t& outBits) {
  const float f = float(value);
  memcpy(&outBits, &f, sizeof(outBits));
  return true;
}

uint64_t CanMessage::doubleToDoubleBits(double value) {
  uint64_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  return bits;
}

bool CanMessage::signalNameExists(const char* name) const {
  if (!name) return false;
  for (uint8_t i = 0; i < m_signalCount; i++) {
    if (m_signals[i] && m_signals[i]->name() && strcmp(m_signals[i]->name(), name) == 0) {
      return true;
    }
  }
  return false;
}

bool CanMessage::fieldFitsMessage(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness) const {
  uint16_t bits[64];
  if (!enumerateFieldBits(startBit, bitLength, endianness, bits, 64)) return false;
  const uint16_t payloadBits = payloadBitLength();
  for (uint16_t i = 0; i < bitLength; i++) {
    if (bits[i] >= payloadBits) return false;
  }
  return true;
}

bool CanMessage::fieldOverlaps(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness) const {
  uint16_t bits[64];
  if (!enumerateFieldBits(startBit, bitLength, endianness, bits, 64)) return true;
  for (uint16_t i = 0; i < bitLength; i++) {
    if (isBitOccupied(bits[i])) return true;
  }
  return false;
}

void CanMessage::markFieldOccupied(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness) {
  uint16_t bits[64];
  if (!enumerateFieldBits(startBit, bitLength, endianness, bits, 64)) return;
  for (uint16_t i = 0; i < bitLength; i++) {
    setBitOccupied(bits[i]);
  }
}

bool CanMessage::enumerateFieldBits(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness, uint16_t* outBits, uint16_t maxOut) const {
  if (!outBits || bitLength == 0 || bitLength > maxOut || bitLength > 64) return false;

  if (endianness == BIG_ENDIAN) {
    uint16_t bit = startBit;
    for (uint16_t i = 0; i < bitLength; i++) {
      outBits[i] = bit;
      bit = nextBigEndianBit(bit);
    }
    return true;
  }

  if (endianness == LITTLE_ENDIAN || bitLength <= 8U) {
    for (uint16_t i = 0; i < bitLength; i++) {
      outBits[i] = uint16_t(startBit + i);
    }
    return true;
  }

  return false;
}


bool CanMessage::constantValueFits(const CanConstantConfig& constantConfig) const {
  if (constantConfig.bitLength >= 64U) return true;
  const uint64_t maxValue = (uint64_t(1) << constantConfig.bitLength) - 1ULL;
  return constantConfig.value <= maxValue;
}

bool CanMessage::fieldBitSequenceValid(uint16_t startBit, uint16_t bitLength, SignalEndianness endianness) const {
  if (bitLength == 0 || bitLength > 64) return false;
  if (bitLength <= 8) return true;
  if (!isExplicitEndianness(endianness)) return false;
  uint16_t bits[64];
  return enumerateFieldBits(startBit, bitLength, endianness, bits, 64);
}

bool CanMessage::isExplicitEndianness(SignalEndianness endianness) {
  return endianness == LITTLE_ENDIAN || endianness == BIG_ENDIAN;
}

uint16_t CanMessage::nextBigEndianBit(uint16_t bit) {
  // DBC/Motorola sawtooth bit numbering: walk down within the byte, then wrap to
  // the MSb of the next byte.
  return (bit % 8U == 0U) ? uint16_t(bit + 15U) : uint16_t(bit - 1U);
}

bool CanMessage::isBitOccupied(uint16_t bit) const {
  if (bit >= 512U) return true;
  const uint8_t word = uint8_t(bit / 64U);
  const uint8_t offset = uint8_t(bit % 64U);
  return (m_occupiedBits[word] & (uint64_t(1) << offset)) != 0;
}

void CanMessage::setBitOccupied(uint16_t bit) {
  if (bit >= 512U) return;
  const uint8_t word = uint8_t(bit / 64U);
  const uint8_t offset = uint8_t(bit % 64U);
  m_occupiedBits[word] |= (uint64_t(1) << offset);
}

void CanMessage::clearOccupancy() {
  for (uint8_t i = 0; i < 8; i++) {
    m_occupiedBits[i] = 0;
  }
}


bool CanMessage::classicFrameAllowed() const {
  if (!m_configValid) return false;
  if (m_config.frameFormat == FRAME_FD) return false;

  // Bit-rate switching is a CAN FD feature. If BRS is requested, the
  // message must not be sent as a classic CAN frame. This also makes
  // FRAME_CLASSIC + bitrateSwitch=true fail cleanly at send time.
  if (m_config.bitrateSwitch) return false;

  return m_config.dlc <= 8U;
}

bool CanMessage::fdFrameAllowed() const {
  if (!m_configValid) return false;
  if (m_config.frameFormat == FRAME_CLASSIC) return false;
  return m_config.dlc <= 15U;
}

bool CanMessage::requiresCanFd() const {
  if (!m_configValid) return false;
  if (m_config.frameFormat == FRAME_FD) return true;
  if (m_config.dlc > 8U) return true;
  if (m_config.bitrateSwitch) return true;
  return false;
}

bool CanMessage::validTransportLength() const {
  if (!m_configValid) return false;
  if (m_config.frameFormat == FRAME_CLASSIC) return classicFrameAllowed();
  if (m_config.frameFormat == FRAME_FD) return fdFrameAllowed();

  // FRAME_AUTO: BRS implies CAN FD. Without BRS, DLC 0..8 may be classic
  // CAN and DLC 9..15 is CAN FD. Since fdFrameAllowed() accepts all valid
  // raw DLC codes 0..15, this is valid whenever either path is valid.
  if (m_config.bitrateSwitch) return fdFrameAllowed();
  return classicFrameAllowed() || fdFrameAllowed();
}

uint8_t CanMessage::dlcToLength(uint8_t dlc) {
  static const uint8_t DLC_TO_LENGTH[16] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 12, 16, 20, 24, 32, 48, 64
  };
  return (dlc <= 15U) ? DLC_TO_LENGTH[dlc] : 0U;
}

bool CanMessage::isValidDlcCode(uint8_t dlc) {
  return dlc <= 15U;
}

const char* CanMessage::errorText() const {
  switch (m_error) {
    case OK: return "OK";
    case NULL_NAME: return "Message name is null or empty";
    case INVALID_ID_TYPE: return "Invalid CAN ID type";
    case INVALID_STANDARD_ID: return "Invalid standard CAN ID";
    case INVALID_EXTENDED_ID: return "Invalid extended CAN ID";
    case INVALID_DLC: return "Invalid DLC";
    case TOO_MANY_SIGNALS: return "Too many signals in message";
    case TOO_MANY_CONSTANTS: return "Too many constants in message";
    case INVALID_SIGNAL: return "Invalid signal";
    case INVALID_FIELD_RANGE: return "Field range does not fit in message";
    case INVALID_ENDIANNESS: return "Invalid endianness";
    case ENDIANNESS_REQUIRED: return "Endianness required for fields wider than 8 bits";
    case FIELD_OVERLAP: return "Field overlaps an existing signal or constant";
    case DUPLICATE_SIGNAL_NAME: return "Duplicate signal name in message";
    case BUFFER_TOO_SMALL: return "Pack/decode buffer is too small";
    case VALUE_OUT_OF_RANGE: return "Value is out of range for field";
    case FRAME_ID_MISMATCH: return "Frame ID does not match message";
    case FRAME_TYPE_MISMATCH: return "Frame type does not match message";
    case FRAME_TOO_SHORT: return "Received frame is shorter than message payload";
    case DECODE_FAILED: return "Message decode failed";
    default: return "Unknown message error";
  }
}

} // namespace CANMessageSignal
