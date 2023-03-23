/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#include "perception/perception_obstacle.hpp"

namespace ADPlanning {
const std::vector<ObstacleInfo> PerceptionObstacle::static_obstacles() const {
  return static_obstacles_;
}

const std::vector<ObstacleInfo> PerceptionObstacle::dynamic_obstacles() const {
  return dynamic_obstacles_;
}

void PerceptionObstacle::UpdateObstacleInfo(
    const int time, const LocalizationInfo& LocalizationInfo) {
  ObstacleInfo obs0;
  obs0.id = 0;
  obs0.x = 400;
  obs0.y = 20;
  obs0.v = 0;
}

void PerceptionObstacle::AddStaticObstacle(const int id, const double init_x,
                                           const double init_y,
                                           const double init_heading,
                                           const double init_v) {
  ObstacleInfo obs;
  if (static_obstacles_.empty()) {
    obs.id = 0;
  } else {
    obs.id = static_obstacles_.size() - 1;
  }
  obs.init_x = init_x;
  obs.init_y = init_y;
  obs.init_heading = init_heading;

  obs.x = init_x;
  obs.y = init_y;
  obs.heading = init_heading;
  obs.v = init_v;

  static_obstacles_.push_back(obs);
}

void PerceptionObstacle::AddDynamicObstacle(const int id, const double init_x,
                                            const double init_y,
                                            const double init_heading,
                                            const double init_v) {
  ObstacleInfo obs;
  if (dynamic_obstacles_.empty()) {
    obs.id = 0;
  } else {
    obs.id = static_obstacles_.size() - 1;
  }
  obs.init_x = init_x;
  obs.init_y = init_y;
  obs.init_heading = init_heading;

  obs.x = init_x;
  obs.y = init_y;
  obs.heading = init_heading;

  obs.v = init_v;
  obs.vx = obs.v * cos(obs.heading);
  obs.vy = obs.v * sin(obs.heading);

  obs.a = 0;
  obs.ax = obs.a * cos(obs.heading);
  obs.ay = obs.a * sin(obs.heading);

  dynamic_obstacles_.push_back(obs);
}

PerceptionObstacle::PerceptionObstacle() {
  static_obstacles_.clear();
  dynamic_obstacles_.clear();
}
}  // namespace ADPlanning