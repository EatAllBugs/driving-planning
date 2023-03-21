/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#pragma once
#include <vector>

#include "common/pnc_point.hpp"

namespace ADPlanning {
class Trajectory {
 public:
  Trajectory();
  ~Trajectory();
  const std::vector<TrajectoryPoint> trajectory_points() const;
  void set_trajectory_points(
      const std::vector<TrajectoryPoint> &trajectory_points);

 private:
  std::vector<TrajectoryPoint> trajectory_points_;
};
}  // namespace ADPlanning