/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */

/*
1.routing 自定义一条ReferenceLine
  location 选择轨迹前进n个点作为定位点
  preception 选取前进方向范围内的障碍物，作为识别的障碍物信息
2.调用参考线平滑模块，对参考线进行平滑
3.调用EMPlanner算法进行Planning
*/

#include <ctime>
#include <memory>

#include "EMPlanner/EMPlanner.hpp"
#include "localization/localization_estimate.hpp"
#include "perception/perception_obstacle.hpp"
#include "reference_line/reference_line_provider.hpp"
#include "routing/routing_path.hpp"

int main(int argc, char const *argv[]) {
  // 构造配置
  std::unique_ptr<ADPlanning::EMPlannerConfig> config =
      std::make_unique<ADPlanning::EMPlannerConfig>();

  // 构造路由模块指针
  std::unique_ptr<ADPlanning::RoutingPath> routing_path =
      std::make_unique<ADPlanning::RoutingPath>();

  // 定位信息指针
  std::unique_ptr<ADPlanning::LocalizationEstimate> localization =
      std::unique_ptr<ADPlanning::LocalizationEstimate>();

  // 障碍物信息
  std::unique_ptr<ADPlanning::PerceptionObstacle> perception =
      std::unique_ptr<ADPlanning::PerceptionObstacle>();

  // 构造全局路径
  routing_path->CreatePath();

  // 创建一个静态障碍物
  perception->AddStaticObstacle(0, 400, 20, 0, 0);

  auto routing_path_points = routing_path->routing_path_points();

  ADPlanning::LocalizationInfo localization_info;
  ADPlanning::ReferenceLine reference_line;
  ADPlanning::ReferenceLine pre_reference_line;

  ADPlanning::Trajectory trajectory;
  ADPlanning::Trajectory pre_trajectory;
  uint64_t time = 0;
  while (1) {
    // 每10ms循环执行一次
    if (time % 10 - 0 < 1e-10) {
      // 更新车辆信息，定位模块更新位置, 取规划轨迹+10ms的位置
      localization->UpdateLocalizationInfo(time, trajectory);
      localization_info = localization->localization_info();
      // 更新障碍物信息
      perception->UpdateObstacleInfo();
    }
    // 每100ms 循环执行一次
    if ((time % 100) < 1e-10) {
      // 参考线生成,参考新默认-30m~150m
      std::unique_ptr<ADPlanning::ReferenceLineProvider>
          reference_line_provider =
              std::unique_ptr<ADPlanning::ReferenceLineProvider>();
      pre_reference_line = reference_line;
      // 传参应该是数据类型，而不是类的对象
      reference_line_provider->Provide(routing_path_points, localization_info,
                                       pre_reference_line, reference_line);
      // 规划器规划
      std::unique_ptr<ADPlanning::EMPlanner> em_planner =
          std::make_unique<ADPlanning::EMPlanner>(*config);
      ADPlanning::TrajectoryPoint plan_start;
      ADPlanning::Trajectory stitch_trajectory;
      pre_trajectory = trajectory;
      em_planner->CalPlaningStartPoint(pre_trajectory, localization_info,
                                       &plan_start, &stitch_trajectory);
      std::vector<ADPlanning::ReferencePoint> xy_virtual_obstacles;
      em_planner->Plan(time, plan_start, reference_line, localization,
                       perception, trajectory, xy_virtual_obstacles);
      perception->UpdateVirtualObstacle(xy_virtual_obstacles);
    }
    time++;
    std::usleep(1000);
  }
  return 0;
}
