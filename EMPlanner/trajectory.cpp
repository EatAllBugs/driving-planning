/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */

#include "EMPlanner/trajectory.hpp"
namespace ADPlanning {

Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}
const std::vector<TrajectoryPoint> Trajectory::trajectory_points() const {
  return trajectory_points_;
}

void Trajectory::set_trajectory_points(
    const std::vector<TrajectoryPoint> &trajectory_points) {
  trajectory_points_ = trajectory_points;
}
}  // namespace ADPlanning