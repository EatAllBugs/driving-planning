/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */

#pragma once

#include <float.h>

#include <algorithm>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"
#include "config/EMPlanner_config.hpp"
#include "eigen3/Eigen/Eigen"
#include "localization/localization_estimate.hpp"
#include "path_time_graph.hpp"
#include "perception/perception_obstacle.hpp"
#include "reference_line/reference_line_provider.hpp"
#include "trajectory.hpp"

namespace ADPlanning {
class SpeedTimeGraph {
 public:
  SpeedTimeGraph(const ReferenceLine &planning_path,
                 const EMPlannerConfig &emplanner_conf);
  ~SpeedTimeGraph() = default;

  /**
   * @brief
   * @return const Trajectory
   */
  const Trajectory trajectory() const;

  /**
   * @brief
   * @return const std::vector<ObstacleInfo>
   */
  const std::vector<ObstacleInfo> xy_virtual_obstacles() const;

  /**
   * @brief
   * @return std::vector<STLine>
   */
  std::vector<STLine> st_obstacles();

  /**
   * @brief
   * @return const std::vector<STPoint>
   */
  const std::vector<STPoint> dp_speed_points() const;

  /**
   * @brief
   * @return const std::vector<STPoint>
   */
  const std::vector<STPoint> qp_speed_points() const;

  /**
   * @brief
   * @return const std::vector<STPoint>
   */
  const std::vector<STPoint> qp_speed_points_dense() const;

  /**
   * @brief 基于规划的轨迹，初始化坐标轴
   * @param planning_path
   */
  void InitSAxis(const ReferenceLine &planning_path);

  /**
   * @brief 计算速度规划的初始状态
   * @param sl_plan_start
   */
  void SetStartState(const SLPoint &sl_plan_start);

  /**
   * @brief 计算障碍物的ST位置
   * @param dynamic_obstacles
   */
  void SetDynamicObstaclesSL(
      const std::vector<ObstacleInfo> &dynamic_obstacles);

  /**
   * @brief 生成STGraph
   */
  void GenerateSTGraph();

  /**
   * @brief Create a Smaple Point object
   * @param row
   * @param col
   */
  void CreateSmaplePoint(const int row, const int col);

  /**
   * @brief
   */
  void SpeedDynamicPlanning();

  /**
   * @brief
   * @param point_s
   * @param point_e
   * @return double
   */
  double CalcDpCost(const STPoint &point_s, const STPoint &point_e);

  /**
   * @brief
   * @param point_s
   * @param point_e
   * @return double
   */
  double CalcObsCost(const STPoint &point_s, const STPoint &point_e);

  /**
   * @brief
   * @param w_cost_obs
   * @param min_dis
   * @return double
   */
  double CalcCollisionCost(const double w_cost_obs, const double min_dis);

  /**
   * @brief
   */
  void GenerateCovexSpace();

  /**
   * @brief
   * @param t
   * @return int
   */
  int FindDpMatchIndex(const double t);

  /**
   * @brief
   * @return bool
   */
  bool SpeedQuadraticProgramming();

  /**
   * @brief
   * @param interpolation_num
   */
  void SpeedQpInterpolation(const int interpolation_num);

  /**
   * @brief path和speed的合并
   */
  void PathAndSpeedMerge();

 private:
  EMPlannerConfig emplaner_conf_;
  ReferenceLine planning_path_;

  std::vector<ReferencePoint> planning_path_points_;
  std::vector<SLPoint> sl_planning_path_;

  std::vector<SLPoint> sl_dynamic_obstacles_;
  SLPoint plan_start_;
  STPoint st_plan_start_;
  std::vector<SLPoint> sl_virtual_obstacles_;
  std::vector<ReferencePoint> xy_virtual_obstacles_;

  std::vector<STLine> st_obstacles_;
  std::vector<std::vector<STPoint>> sample_points_;

  std::vector<STPoint> dp_speed_points_;
  std::vector<STPoint> dp_speed_points_dense_;

  Eigen::VectorXd convex_s_lb_;
  Eigen::VectorXd convex_s_ub_;
  Eigen::VectorXd convex_ds_dt_lb_;
  Eigen::VectorXd convex_ds_dt_ub_;

  std::vector<STPoint> qp_speed_points_;
  std::vector<STPoint> qp_speed_points_dense_;

  std::vector<TrajectoryPoint> trajectory_points_;
  Trajectory trajectory_;
};
}  // namespace ADPlanning
