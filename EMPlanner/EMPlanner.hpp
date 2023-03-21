/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#pragma once

#include <memory>

#include "EMPlanner/path_time_graph.hpp"
#include "EMPlanner/speed_time_graph.hpp"
#include "config/EMPlanner_config.hpp"
#include "localization/localization_estimate.hpp"
#include "perception/perception_obstacle.hpp"
#include "reference_line/reference_line.hpp"
#include "trajectory.hpp"

namespace ADPlanning {
class EMPlanner {
 public:
  EMPlanner(const EMPlannerConfig &conf);
  ~EMPlanner() = default;

  /**
   * @brief 轨迹规划函数入口
   * @param current_time
   * @param planning_init_point
   * @param reference_line
   * @param localization_info
   * @param static_obstacles
   * @param dynamic_obstacles
   * @param trajectory
   * @param xy_virtual_obstacles
   */
  void Plan(const uint64_t current_time,
            const TrajectoryPoint &planning_init_point,
            const ReferenceLine &reference_line,
            const LocalizationInfo &localization,
            const std::vector<ObstacleInfo> &obstacle,
            const std::vector<ObstacleInfo> &dynamic_obstacles,
            Trajectory *trajectory,
            std::vector<ObstacleInfo> &xy_virtual_obstacles);

  /**
   * @brief 该函数将计算规划的起点以及拼接轨迹的信息输入,
   * 当规划完成后，本周期的规划结果和stitch_trajectory一起拼好发给控制
   * @param pre_traj
   * @param local_info
   * @param plan_start_point
   * @param stitch_traj
   */
  void CalPlaningStartPoint(const Trajectory &pre_traj,
                            const LocalizationInfo &local_info,
                            TrajectoryPoint *start_plan_point,
                            Trajectory *stitch_traj);
  /**
   * @brief 轨迹拼接函数
   * @param cur_traj 当前轨迹
   * @param stitch_traj 拼接轨迹
   * @param final_traj 最终输出轨迹
   */
  void StitchTrajectory(const Trajectory &cur_traj,
                        const Trajectory &stitch_traj, Trajectory &final_traj);

  const std::unique_ptr<PathTimeGraph> SLGraph() const;
  const std::unique_ptr<SpeedTimeGraph> STGraph() const;

 private:
  EMPlannerConfig config_;
  Trajectory trajectory_;
  Trajectory pre_trajectory_;
  std::unique_ptr<PathTimeGraph> sl_graph_;
  std::unique_ptr<SpeedTimeGraph> st_graph_;
};

}  // namespace ADPlanning
