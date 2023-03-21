/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */

#pragma once
namespace ADPlanning {
class ReferenceLineSmootherConfig {
 public:
  double weight_smooth = 70.0;
  double weight_path_length = 10.0;
  double weight_ref_deviation = 20.0;
  double x_lower_bound = -2.0;
  double x_upper_bound = 2.0;
  double y_lower_bound = -2.0;
  double y_upper_bound = 2.0;
};
}  // namespace ADPlanning