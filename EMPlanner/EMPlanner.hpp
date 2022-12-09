/*
此类的功能：
通过动态规划和二次规划方法输出轨迹
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

  void Plan(const uint64_t current_time,                 // 当前时间
            const TrajectoryPoint &planning_init_point,  // 规划起点
            const ReferenceLine &reference_line,         // 参考线
            const LocalizationInfo &localization,        // 定位信息
            const std::vector<ObstacleInfo> &obstacle,   // 障碍物信息
            const std::vector<ObstacleInfo> &dynamic_obstacles,
            Trajectory *trajectory,                           // 输出轨迹
            std::vector<ObstacleInfo> xy_virtual_obstacles);  // 虚拟障碍物

  void CalPlaningStartPoint(const Trajectory &pre_traj,
                            const LocalizationInfo &local_info,
                            TrajectoryPoint *start_plan_point,
                            Trajectory *stitch_traj);

  void StitchTrajectory(const Trajectory &cur_traj,
                        const Trajectory &stitch_traj, Trajectory &final_traj);

  // const std::unique_ptr<PathTimeGraph> sl_graph() const;
  // const std::unique_ptr<SpeedTimeGraph> st_graph() const;

  std::unique_ptr<PathTimeGraph> sl_graph_;
  std::unique_ptr<SpeedTimeGraph> st_graph_;

 private:
  EMPlannerConfig config_;

  Trajectory pre_trajectory_;
  Trajectory trajectory_;
};

}  // namespace ADPlanning
