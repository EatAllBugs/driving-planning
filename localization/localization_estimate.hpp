/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */

#pragma once

#include <stdint.h>

#include <cinttypes>

#include "EMPlanner/trajectory.hpp"
namespace ADPlanning {
class LocalizationEstimate {
 public:
  LocalizationEstimate();
  ~LocalizationEstimate() = default;

  void UpdateLocalizationInfo(const uint64_t time,
                              const Trajectory &trajectory);
  const LocalizationInfo localization_info() const;

 private:
  LocalizationInfo localization_info_;
};
}  // namespace ADPlanning