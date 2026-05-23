#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <ACANFD_SAME-from-cpp.h>
#include "CanMessage.h"

namespace CANMessageSignal {

enum CanBusType : uint8_t {
  CAN_CLASSIC = 0,
  CAN_FD = 1
};

class CanChannel {
public:
  enum ErrorCode : uint8_t {
    OK = 0,
    NULL_CONTROLLER,
    INVALID_MESSAGE,
    INVALID_TRANSPORT_LENGTH,
    PACK_FAILED,
    SEND_FAILED,
    CAN_FD_NOT_ALLOWED_ON_CLASSIC_BUS,
    BRS_REQUIRES_CAN_FD
  };

  CanChannel() = default;
  explicit CanChannel(ACANFD_SAME& controller);

  void attach(ACANFD_SAME& controller);
  bool isAttached() const { return m_controller != nullptr; }

  // Optional safety guard. Default is CAN_FD, which allows both classic CAN
  // frames and CAN FD frames. Set CAN_CLASSIC to reject accidental CAN FD
  // transmit attempts before they reach the ACANFD driver.
  void setBusType(CanBusType busType);
  CanBusType busType() const { return m_busType; }

  bool sendMessage(CanMessage& message);

  // Must be called repeatedly, for example from loop().
  // Sends only when the specified period has elapsed.
  // Returns true only when a send was attempted and accepted by the driver.
  // Returns false when the message is not due yet, or when a due send failed.
  // Check hasError()/lastError() to distinguish a real failure from "not due".
  bool sendIfDue(CanMessage& message, uint32_t periodMs);

  ErrorCode errorCode() const { return m_error; }
  ErrorCode lastError() const { return m_error; }
  bool hasError() const { return m_error != OK; }
  void clearError();
  const char* errorText() const;
  uint32_t lastDriverStatus() const { return m_lastDriverStatus; }

private:
  ACANFD_SAME* m_controller = nullptr;
  ErrorCode m_error = OK;
  uint32_t m_lastDriverStatus = 0;
  CanBusType m_busType = CAN_FD;

  bool buildClassicFrame(CanMessage& message, CANMessage& outFrame);
  bool buildFDFrame(CanMessage& message, CANFDMessage& outFrame);
  static bool shouldSendClassic(const CanMessage& message);
  static CANFDMessage::Type fdFrameTypeForMessage(const CanMessage& message);
};

} // namespace CANMessageSignal
