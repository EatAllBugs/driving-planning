/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#pragma once

namespace ADPlanning {
class EMPlannerConfig {
 public:
  EMPlannerConfig() = default;
  ~EMPlannerConfig() = default;
  double planning_time = 8.0;

  double dp_cost_collision = 1.8;
  double dp_cost_dl = 120.0;
  double dp_cost_ddl = 1.0;
  double dp_cost_dddl = 1.0;
  double dp_cost_ref = 20.0;

  double dp_row = 1.0;
  double dp_col = 1.0;

  double qp_cost_l = 10.0;
  double qp_cost_dl = 1.0;
  double qp_cost_ddl = 1.0;
  double qp_cost_dddl = 1.0;
  double qp_cost_centre = 5.0;
  double qp_cost_end_l = 1.0;
  double qp_cost_end_dl = 1.0;
  double qp_cost_end_ddl = 1.0;

  double ref_speed = 10.0;
  double speed_dp_cost_ref_speed = 100.0;
  double speed_dp_cost_accel = 10.0;
  double speed_dp_cost_obs = 1.0;

  double speed_qp_cost_v_ref = 8.0;
  double speed_qp_cost_dds_dt = 5.0;
  double speed_qp_cost_jerk = 0.0;

  double max_lateral_accel = 1.0;
};
}  // namespace ADPlanning