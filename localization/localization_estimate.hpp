#pragma once

#include <stdint.h>

#include <cinttypes>

#include "EMPlanner/trajectory.hpp"
namespace ADPlanning {
class LocalizationEstimate {
 private:
  /* data */
  LocalizationInfo localization_info_;

 public:
  LocalizationEstimate(/* args */);
  ~LocalizationEstimate() = default;

  void UpdateLocalizationInfo(uint64_t time, Trajectory trajectory);
  const LocalizationInfo localization_info() const;
};
}  // namespace ADPlanning