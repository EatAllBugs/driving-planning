/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#pragma once

#include <vector>

#include "common/pnc_point.hpp"
namespace ADPlanning {
class RoutingPath {
 public:
  const std::vector<MapPoint> routing_path_points() const;

  void set_routing_path(const std::vector<MapPoint> &routing_path_point);
  void CreatePath();
  std::vector<MapPoint> GetRoutingPathFromCSV();

 private:
  std::vector<MapPoint> routing_path_points_;
};
}  // namespace ADPlanning