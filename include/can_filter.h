#pragma once
#include <stdint.h>

struct FilterRule {
  uint32_t identifier = 0;
  bool extended = false;
  bool active = false;
};

inline bool matchesRule(const FilterRule &rule, uint32_t identifier, bool extended) {
  return rule.active && rule.extended == extended && rule.identifier == identifier;
}