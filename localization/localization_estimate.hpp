#pragma once

#include <cmath>

#include "EMPlanner/trajectory.hpp"
namespace ADPlanning {
class LocalizationEstimate {
 private:
  /* data */
  LocalizationInfo localization_info_;

 public:
  LocalizationEstimate(/* args */);
  ~LocalizationEstimate() = default;

  void UpdateLocalizationInfo(u_int64_t time, Trajectory trajectory);
  const LocalizationInfo localization_info() const;
};
}  // namespace ADPlanning