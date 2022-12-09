#include <unistd.h>

#include <memory>

#include "EMPlanner/EMPlanner.hpp"
#include "localization/localization_estimate.hpp"
#include "perception/perception_obstacle.hpp"
#include "plot/plot.hpp"
#include "reference_line/reference_line_provider.hpp"
#include "routing/routing_path.hpp"

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char const *argv[]) {
  std::vector<double> x = {0};
  std::vector<double> y = {0};

  plt::plot(x, y);

  std::unique_ptr<ADPlanning::RoutingPath> routing_path =
      std::make_unique<ADPlanning::RoutingPath>();

  std::unique_ptr<ADPlanning::LocalizationEstimate> localization =
      std::make_unique<ADPlanning::LocalizationEstimate>();

  std::unique_ptr<ADPlanning::PerceptionObstacle> perception =
      std::make_unique<ADPlanning::PerceptionObstacle>();

  routing_path->CreatePath();

  perception->AddStaticObstacle(0, 30, -0.5, 0, 0);

  perception->AddDynamicObstacle(1, 40, -10, M_PI / 2, 2);

  auto static_obstacle_info = perception->static_obstacles();
  auto dynamic_obstacle_info = perception->dynamic_obstacles();

  auto routing_path_points = routing_path->routing_path_points();

  ADPlanning::LocalizationInfo localization_info;

  ADPlanning::ReferenceLine reference_line;
  std::vector<ADPlanning::ReferencePoint> rp;
  rp.clear();
  reference_line.set_reference_points(rp);

  ADPlanning::ReferenceLine pre_reference_line;  // 上一时刻参考线

  ADPlanning::Trajectory trajectory;
  ADPlanning::Trajectory pre_trajectory;
  uint64_t time = 0;

  localization->UpdateLocalizationInfo(0, trajectory);
  localization_info = localization->localization_info();

  // 2.参考线生成,参考新默认-30m~150m
  std::unique_ptr<ADPlanning::ReferenceLineProvider> reference_line_provider =
      std::make_unique<ADPlanning::ReferenceLineProvider>();
  pre_reference_line = reference_line;
  // 传参应该是数据类型，而不是类的对象
  reference_line_provider->Provide(routing_path_points, localization_info,
                                   pre_reference_line, reference_line);
  auto raw_reference_line = reference_line_provider->raw_reference_line();

  ADPlanning::EMPlannerConfig emplanner_config;
  std::unique_ptr<ADPlanning::EMPlanner> em_planner =
      std::make_unique<ADPlanning::EMPlanner>(emplanner_config);

  ADPlanning::TrajectoryPoint plan_start_point;
  ADPlanning::Trajectory stitch_trajectory;

  std::vector<ADPlanning::ObstacleInfo> virtual_obs;

  em_planner->CalPlaningStartPoint(pre_trajectory, localization_info,
                                   &plan_start_point, &stitch_trajectory);

  em_planner->Plan(0, plan_start_point, raw_reference_line, localization_info,
                   static_obstacle_info, dynamic_obstacle_info, &trajectory,
                   virtual_obs);

  std::unique_ptr<ADPlanning::Plot> plot = std::make_unique<ADPlanning::Plot>();

  plt::figure(1);  // xy
  plot->PlotRoutingPath(routing_path_points, "k");
  plot->PlotReferenceLine(reference_line, "y");

  plt::figure(2);  // sl
  plot->PlotSLPath(em_planner->sl_graph_->dp_path_points(), "r");
  plot->PlotSLPath(em_planner->sl_graph_->dp_path_points_dense(), "g");

  for (const auto obs : static_obstacle_info) {
    plot->PlotObs(obs, "k");
  }

  plot->PlotSLPath(em_planner->sl_graph_->qp_path_points(), "r");

  plot->PlotSLPath(em_planner->sl_graph_->qp_path_points_dense(), "p");

  plot->PlotPlanningPath(
      em_planner->sl_graph_->planning_path().reference_points(), "b");

  plt::figure(3);  // st
  plot->PlotSTObs(em_planner->st_graph_->st_obstacles(), "k");
  plot->PlotSTPath(em_planner->st_graph_->dp_speed_points(), "r");
  plt::figure(3);
  plot->PlotSTPath(em_planner->st_graph_->qp_speed_points(), "b");

  plt::figure(3);
  plot->PlotSTPath(em_planner->st_graph_->qp_speed_points_dense(), "g");

  plt::show();
  return 0;
}