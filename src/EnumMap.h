#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <initializer_list>

#ifndef CANMS_ENUMMAP_MAX_ENTRIES
#define CANMS_ENUMMAP_MAX_ENTRIES 32
#endif

namespace CANMessageSignal {

struct EnumMapEntry {
  int64_t value;
  const char* label;
};

class EnumMap {
public:
  enum ErrorCode : uint8_t {
    OK = 0,
    TOO_MANY_ENTRIES,
    DUPLICATE_VALUE,
    NULL_LABEL
  };

  EnumMap() = default;
  EnumMap(std::initializer_list<EnumMapEntry> entries) {
    load(entries);
  }

  bool load(std::initializer_list<EnumMapEntry> entries);

  bool isValid() const { return m_error == OK; }
  ErrorCode errorCode() const { return m_error; }
  const char* errorText() const;

  uint8_t count() const { return m_count; }
  bool containsValue(int64_t value) const;
  const char* labelFor(int64_t value) const;
  bool valueForLabel(const char* label, int64_t& outValue) const;

  bool entryAt(uint8_t index, EnumMapEntry& out) const;
  int64_t valueAt(uint8_t index) const;
  const char* labelAt(uint8_t index) const;

private:
  EnumMapEntry m_entries[CANMS_ENUMMAP_MAX_ENTRIES] = {};
  uint8_t m_count = 0;
  ErrorCode m_error = OK;

  void clear();
  bool hasDuplicateValue(int64_t value) const;
};

} // namespace CANMessageSignal
