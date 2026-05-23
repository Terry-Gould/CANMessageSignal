#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include "CanMessage.h"

namespace CANMessageSignal {

// Fixed-capacity message database / dispatcher.
// The user remains responsible for CAN driver receive setup, filters, FIFOs,
// callbacks, interrupts, etc. The database only receives a frame object and
// tries to decode it against the registered message definitions.
//
// The database is intentionally not channel-aware. It matches by CAN ID and
// standard/extended ID type. For dual-channel applications with duplicate IDs
// on different buses, use one database per channel.
template <uint8_t MAX_MESSAGES>
class CanMessageDatabase {
public:
  enum ErrorCode : uint8_t {
    OK = 0,
    TOO_MANY_MESSAGES,
    NULL_MESSAGE,
    DUPLICATE_MESSAGE_ID,
    NO_MATCH,
    DECODE_FAILED
  };

  CanMessageDatabase() = default;

  bool addMessage(CanMessage& message) {
    if (!message.isValid()) {
      m_error = NULL_MESSAGE;
      return false;
    }

    if (m_count >= MAX_MESSAGES) {
      m_error = TOO_MANY_MESSAGES;
      return false;
    }

    for (uint8_t i = 0; i < m_count; i++) {
      if (m_messages[i] != nullptr &&
          m_messages[i]->id() == message.id() &&
          m_messages[i]->idType() == message.idType()) {
        m_error = DUPLICATE_MESSAGE_ID;
        return false;
      }
    }

    m_messages[m_count++] = &message;
    m_lastDecodedMessage = nullptr;
    m_error = OK;
    return true;
  }

  bool decode(const CANMessage& frame) {
    m_lastDecodedMessage = nullptr;

    for (uint8_t i = 0; i < m_count; i++) {
      CanMessage* msg = m_messages[i];
      if (msg == nullptr) {
        continue;
      }

      const bool headerMatches =
        (msg->id() == frame.id) &&
        (frame.ext == (msg->idType() == EXTENDED)) &&
        !frame.rtr;

      if (!headerMatches) {
        continue;
      }

      if (msg->decode(frame)) {
        m_lastDecodedMessage = msg;
        m_error = OK;
        return true;
      }

      m_lastDecodedMessage = msg;
      m_error = DECODE_FAILED;
      return false;
    }

    // No match is a normal result on a real CAN bus.
    // The database may only contain the messages the user cares about.
    // Unknown frames should return false without setting an error.
    m_error = OK;
    return false;
  }

  bool decode(const CANFDMessage& frame) {
    m_lastDecodedMessage = nullptr;

    for (uint8_t i = 0; i < m_count; i++) {
      CanMessage* msg = m_messages[i];
      if (msg == nullptr) {
        continue;
      }

      const bool headerMatches =
        (msg->id() == frame.id) &&
        (frame.ext == (msg->idType() == EXTENDED)) &&
        (frame.type != CANFDMessage::CAN_REMOTE);

      if (!headerMatches) {
        continue;
      }

      if (msg->decode(frame)) {
        m_lastDecodedMessage = msg;
        m_error = OK;
        return true;
      }

      m_lastDecodedMessage = msg;
      m_error = DECODE_FAILED;
      return false;
    }

    // No match is a normal result on a real CAN bus.
    // The database may only contain the messages the user cares about.
    // Unknown frames should return false without setting an error.
    m_error = OK;
    return false;
  }

  uint8_t count() const { return m_count; }

  CanMessage* messageAt(uint8_t index) {
    return (index < m_count) ? m_messages[index] : nullptr;
  }

  const CanMessage* messageAt(uint8_t index) const {
    return (index < m_count) ? m_messages[index] : nullptr;
  }

  CanMessage* lastDecodedMessage() { return m_lastDecodedMessage; }
  const CanMessage* lastDecodedMessage() const { return m_lastDecodedMessage; }

  bool hasError() const { return m_error != OK; }
  ErrorCode lastError() const { return m_error; }
  ErrorCode errorCode() const { return m_error; }

  void clearError() {
    m_error = OK;
    m_lastDecodedMessage = nullptr;
  }

  const char* errorText() const {
    switch (m_error) {
      case OK: return "OK";
      case TOO_MANY_MESSAGES: return "Too many messages in database";
      case NULL_MESSAGE: return "Invalid message";
      case DUPLICATE_MESSAGE_ID: return "Duplicate message ID in database";
      case NO_MATCH: return "No matching message";
      case DECODE_FAILED:
        return (m_lastDecodedMessage != nullptr) ? m_lastDecodedMessage->errorText() : "Message decode failed";
      default: return "Unknown database error";
    }
  }

private:
  CanMessage* m_messages[MAX_MESSAGES] = {};
  uint8_t m_count = 0;
  CanMessage* m_lastDecodedMessage = nullptr;
  ErrorCode m_error = OK;
};

} // namespace CANMessageSignal
