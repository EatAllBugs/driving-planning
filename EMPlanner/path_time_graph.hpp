/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#include <float.h>

#include <algorithm>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"
#include "common/pnc_point.hpp"
#include "config/EMPlanner_config.hpp"
#include "eigen3/Eigen/Eigen"
#include "localization/localization_estimate.hpp"
#include "perception/perception_obstacle.hpp"
#include "reference_line/reference_line_provider.hpp"

namespace ADPlanning {
class PathTimeGraph {
 public:
  PathTimeGraph(const ReferenceLine &reference_line,
                const EMPlannerConfig &emplaner_conf);
  ~PathTimeGraph() = default;

  /**
   * @brief 笛卡尔坐标系转自然坐标系
   * @param reference_line
   * @param sl_reference_line
   * @param points_wcs
   * @param match_points
   * @param project_points
   * @param points_fcs
   */
  static void Cartesian2Frenet(
      const ReferenceLine &reference_line,
      const std::vector<SLPoint> &sl_reference_line,
      const std::vector<TrajectoryPoint> &points_wcs,
      const std::vector<ReferencePoint> &match_points,
      const std::vector<ReferencePoint> &project_points,
      std::vector<SLPoint> &points_fcs);

  /**
   * @brief 自然坐标系转笛卡尔坐标系
   * @param reference_line
   * @param sl_reference_line
   * @param points_fcs
   * @param points_wcs
   */
  static void Frenet2Cartesian(const ReferenceLine &reference_line,
                               const std::vector<SLPoint> &sl_reference_line,
                               const std::vector<SLPoint> &points_fcs,
                               std::vector<ReferencePoint> *points_wcs);

  /**
   * @brief 根据参考线和自车投影点生成s轴
   * @param reference_line
   * @param sl_reference_line
   */
  void InitSAxis(const ReferenceLine &reference_line,
                 std::vector<SLPoint> *sl_reference_line);

  /**
   * @brief Set the Start Point Sl object
   * @param plan_start_point
   */
  void SetStartPointSl(const TrajectoryPoint &plan_start_point);

  /**
   * @brief Set the Static Obstacles Sl object
   * @param static_obstacles
   */
  void SetStaticObstaclesSl(const std::vector<ObstacleInfo> &static_obstacles);

  /**
   * @brief Create a Sampling Point object
   * @param row
   * @param col
   * @param sample_s
   * @param sample_l
   */
  void CreateSamplingPoint(const int row, const int col, const double sample_s,
                           const double sample_l);

  /**
   * @brief 计算路径cost
   * @param point1
   * @param point2
   * @return double
   */
  double CalcPathCost(const SLPoint &point1, const SLPoint &point2);

  /**
   * @brief
   * @param point1
   * @param point2
   * @param QuinticCoeffient
   */
  void CalcQuinticCoeffient(const SLPoint &point1, const SLPoint &point2,
                            std::vector<double> *QuinticCoeffient);

  /**
   * @brief 横向路径动态规划
   */
  void PathDynamicPlanning();

  /**
   * @brief dp路径插值
   * @param interpolation_num
   * @param ds
   */
  void DpPathInterpolation(const int interpolation_num, const double ds);

  /**
   * @brief 动态规划路径点
   * @return const std::vector<SLPoint>
   */
  const std::vector<SLPoint> dp_path_points() const;

  /**
   * @brief 动态规划稠密路径点
   * @return const std::vector<SLPoint>
   */
  const std::vector<SLPoint> dp_path_points_dense() const;

  /**
   * @brief 动态规划路径点
   * @return const std::vector<SLPoint>
   */
  const std::vector<SLPoint> qp_path_points() const;

  /**
   * @brief 动态规划稠密路径点
   * @return const std::vector<SLPoint>
   */
  const std::vector<SLPoint> qp_path_points_dense() const;

  /**
   * @brief 根据dp_path,输出l_minx,l_max
   * @param static_obs_length
   * @param static_obs_width
   */
  void GenerateConvexSpace(const double static_obs_length,
                           const double static_obs_width);

  /**
   * @brief
   * @param dp_path_points_dense
   * @param s
   * @return int
   */
  int FindNearIndex(const std::vector<SLPoint> &dp_path_points_dense,
                    const double s);

  /**
   * @brief 二次规划
   * @return bool
   */
  bool PathQuadraticProgramming();

  /**
   * @brief 对路径进行插值处理
   * @param interpolation_num
   * @param ds
   */
  void QpPathInterpolation(const int interpolation_num, const double ds);

  /**
   * @brief 生成横向规划路径
   */
  void GeneratePlanningPath();

  /**
   * @brief
   * @param sl_point
   * @param sl_reference_line
   * @param reference_line
   * @param project_point
   */
  void CalcProjPoint(const SLPoint &sl_point,
                     const std::vector<SLPoint> &sl_reference_line,
                     const std::vector<ReferencePoint> &reference_line,
                     ReferencePoint &project_point);
  /**
   * @brief
   * @return const ReferenceLine
   */
  const ReferenceLine planning_path() const;

  /**
   * @brief
   * @return const SLPoint
   */
  const SLPoint sl_plan_start() const;

 private:
  EMPlannerConfig config_;

  ReferenceLine reference_line_;
  ReferencePoint host_match_point_;
  ReferencePoint host_project_point_;
  LocalizationEstimate localization_;
  std::vector<SLPoint> sl_reference_line_;

  SLPoint sl_plan_start_;
  SLPoint sl_host_;
  std::vector<SLPoint> sl_static_obstacles_;
  std::vector<std::vector<SLPoint>> sample_points_;
  std::vector<SLPoint> dp_path_points_;
  std::vector<SLPoint> dp_path_points_dense_;
  std::vector<SLPoint> qp_path_points_;
  std::vector<SLPoint> qp_path_points_dense_;

  ReferenceLine planning_path_;

  Eigen::VectorXd l_min_;
  Eigen::VectorXd l_max_;
};
}  // namespace ADPlanning
