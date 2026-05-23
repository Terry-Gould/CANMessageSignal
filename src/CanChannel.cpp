#include "CanChannel.h"
#include <string.h>

namespace CANMessageSignal {

CanChannel::CanChannel(ACANFD_SAME& controller) {
  attach(controller);
}

void CanChannel::attach(ACANFD_SAME& controller) {
  m_controller = &controller;
  m_error = OK;
  m_lastDriverStatus = 0;
}

void CanChannel::setBusType(CanBusType busType) {
  m_busType = busType;
}

void CanChannel::clearError() {
  m_error = OK;
  m_lastDriverStatus = 0;
}

bool CanChannel::sendIfDue(CanMessage& message, uint32_t periodMs) {
  const uint32_t now = millis();
  if (!message.hasBeenSent() || (uint32_t)(now - message.lastSentMs()) >= periodMs) {
    return sendMessage(message);
  }
  m_error = OK;
  return false;
}

bool CanChannel::sendMessage(CanMessage& message) {
  if (m_controller == nullptr) {
    m_error = NULL_CONTROLLER;
    return false;
  }

  if (message.frameFormat() == FRAME_CLASSIC && message.bitrateSwitch()) {
    m_lastDriverStatus = 0;
    m_error = BRS_REQUIRES_CAN_FD;
    return false;
  }

  if (m_busType == CAN_CLASSIC && message.requiresCanFd()) {
    m_lastDriverStatus = 0;
    m_error = CAN_FD_NOT_ALLOWED_ON_CLASSIC_BUS;
    return false;
  }

  if (shouldSendClassic(message)) {
    CANMessage frame;
    if (!buildClassicFrame(message, frame)) {
      return false;
    }
    // Deliberately pass a classic CANMessage here. ACANFD converts this to a
    // CANFDMessage internally, but keeping the public classic path explicit
    // matches the known-good ACANFD examples and avoids surprising CAN-FD
    // defaults in user sketches.
    m_lastDriverStatus = m_controller->tryToSendReturnStatusFD(frame);
  } else {
    CANFDMessage frame;
    if (!buildFDFrame(message, frame)) {
      return false;
    }
    m_lastDriverStatus = m_controller->tryToSendReturnStatusFD(frame);
  }

  if (m_lastDriverStatus != 0) {
    m_error = SEND_FAILED;
    return false;
  }

  message.markSent(millis());
  m_error = OK;
  return true;
}

bool CanChannel::buildClassicFrame(CanMessage& message, CANMessage& outFrame) {
  if (!message.isValid()) {
    m_error = INVALID_MESSAGE;
    return false;
  }

  if (!message.classicFrameAllowed()) {
    m_error = INVALID_TRANSPORT_LENGTH;
    return false;
  }

  uint8_t payload[64];
  memset(payload, 0, sizeof(payload));
  if (!message.pack(payload, sizeof(payload))) {
    m_error = PACK_FAILED;
    return false;
  }

  outFrame.id = message.id();
  outFrame.ext = (message.idType() == EXTENDED);
  outFrame.rtr = false;
  outFrame.idx = 0;
  outFrame.len = message.lengthBytes();
  if (outFrame.len > 0) {
    memcpy(outFrame.data, payload, outFrame.len);
  }

  m_error = OK;
  return true;
}

bool CanChannel::buildFDFrame(CanMessage& message, CANFDMessage& outFrame) {
  if (!message.isValid()) {
    m_error = INVALID_MESSAGE;
    return false;
  }

  if (!message.fdFrameAllowed()) {
    m_error = INVALID_TRANSPORT_LENGTH;
    return false;
  }

  uint8_t payload[64];
  memset(payload, 0, sizeof(payload));
  if (!message.pack(payload, sizeof(payload))) {
    m_error = PACK_FAILED;
    return false;
  }

  outFrame.id = message.id();
  outFrame.ext = (message.idType() == EXTENDED);
  outFrame.idx = 0;
  outFrame.len = message.lengthBytes();
  outFrame.type = fdFrameTypeForMessage(message);
  if (outFrame.len > 0) {
    memcpy(outFrame.data, payload, outFrame.len);
  }

  m_error = OK;
  return true;
}

bool CanChannel::shouldSendClassic(const CanMessage& message) {
  if (message.frameFormat() == FRAME_CLASSIC) return true;
  if (message.frameFormat() == FRAME_FD) return false;

  // FRAME_AUTO:
  //   - DLC 0..8 with no BRS is classic CAN by default.
  //   - DLC 0..8 with BRS must be CAN FD, because BRS is only valid for CAN FD.
  //   - DLC 9..15 must be CAN FD because classic CAN cannot carry those DLC codes.
  return (message.dlc() <= 8U) && !message.bitrateSwitch();
}

CANFDMessage::Type CanChannel::fdFrameTypeForMessage(const CanMessage& message) {
  return message.bitrateSwitch() ? CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                                 : CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
}

const char* CanChannel::errorText() const {
  switch (m_error) {
    case OK: return "OK";
    case NULL_CONTROLLER: return "No CAN controller attached";
    case INVALID_MESSAGE: return "Invalid CAN message";
    case INVALID_TRANSPORT_LENGTH: return "Invalid CAN/CAN FD transport length";
    case PACK_FAILED: return "Message packing failed";
    case SEND_FAILED: return "CAN driver send failed";
    case CAN_FD_NOT_ALLOWED_ON_CLASSIC_BUS: return "CAN FD frame not allowed on classic CAN bus";
    case BRS_REQUIRES_CAN_FD: return "BRS requires CAN FD frame";
    default: return "Unknown error";
  }
}

} // namespace CANMessageSignal
