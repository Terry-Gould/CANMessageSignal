#include "EnumMap.h"
#include <string.h>

namespace CANMessageSignal {

void EnumMap::clear() {
  m_count = 0;
  m_error = OK;
  for (uint8_t i = 0; i < CANMS_ENUMMAP_MAX_ENTRIES; i++) {
    m_entries[i] = {0, nullptr};
  }
}

bool EnumMap::load(std::initializer_list<EnumMapEntry> entries) {
  clear();

  if (entries.size() > CANMS_ENUMMAP_MAX_ENTRIES) {
    m_error = TOO_MANY_ENTRIES;
    return false;
  }

  for (const auto& entry : entries) {
    if (entry.label == nullptr) {
      m_error = NULL_LABEL;
      return false;
    }

    if (hasDuplicateValue(entry.value)) {
      m_error = DUPLICATE_VALUE;
      return false;
    }

    m_entries[m_count++] = entry;
  }

  m_error = OK;
  return true;
}

bool EnumMap::hasDuplicateValue(int64_t value) const {
  for (uint8_t i = 0; i < m_count; i++) {
    if (m_entries[i].value == value) {
      return true;
    }
  }
  return false;
}

bool EnumMap::containsValue(int64_t value) const {
  return labelFor(value) != nullptr;
}

const char* EnumMap::labelFor(int64_t value) const {
  for (uint8_t i = 0; i < m_count; i++) {
    if (m_entries[i].value == value) {
      return m_entries[i].label;
    }
  }
  return nullptr;
}

bool EnumMap::valueForLabel(const char* label, int64_t& outValue) const {
  if (label == nullptr) {
    return false;
  }

  for (uint8_t i = 0; i < m_count; i++) {
    if (m_entries[i].label != nullptr && strcmp(m_entries[i].label, label) == 0) {
      outValue = m_entries[i].value;
      return true;
    }
  }

  return false;
}

bool EnumMap::entryAt(uint8_t index, EnumMapEntry& out) const {
  if (index >= m_count) {
    return false;
  }
  out = m_entries[index];
  return true;
}

int64_t EnumMap::valueAt(uint8_t index) const {
  if (index >= m_count) {
    return 0;
  }
  return m_entries[index].value;
}

const char* EnumMap::labelAt(uint8_t index) const {
  if (index >= m_count) {
    return nullptr;
  }
  return m_entries[index].label;
}

const char* EnumMap::errorText() const {
  switch (m_error) {
    case OK: return "OK";
    case TOO_MANY_ENTRIES: return "Too many enum entries";
    case DUPLICATE_VALUE: return "Duplicate enum value";
    case NULL_LABEL: return "Null enum label";
    default: return "Unknown enum error";
  }
}

} // namespace CANMessageSignal
