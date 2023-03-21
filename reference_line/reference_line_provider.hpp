#pragma once

/*此类的功能主要为EMPlanner提供参考线*/
#include <cmath>
#include <memory>

#include "reference_line/reference_line_smoother.hpp"
namespace ADPlanning {
class ReferenceLineProvider {
 public:
  ReferenceLineProvider();
  ~ReferenceLineProvider() = default;

  /**
   * @brief
   * 类的主功能函数，由全局路径，定位信息，上一时刻的参考线信息，生成新的参考线
   * @param routing_path_points
   * @param localzation_info
   * @param pre_reference_line
   * @param reference_line
   */
  void Provide(const std::vector<MapPoint> &routing_path_points,
               const LocalizationInfo &localzation_info,
               const ReferenceLine &pre_reference_line,
               ReferenceLine &reference_line);
  const ReferenceLine raw_reference_line() const;
  const ReferenceLine smoothed_reference_line() const;

  /**
   * @brief
   * @param frenet_path
   * @param map_points
   * @param index_start_search
   * @param increase_count_limit
   * @param match_points
   * @param project_points
   */
  static void FindMatchAndProjectPoint(
      const ReferenceLine &frenet_path, const std::vector<MapPoint> &map_points,
      const int index_start_search, const int increase_count_limit,
      std::vector<ReferencePoint> &match_points,
      std::vector<ReferencePoint> &project_points);

 private:
  /**
   * @brief 首次截取调用参考线平滑器，进行参考线平滑
   * @param frenet_path
   * @param host_match_point_index
   * @param reference_line
   */
  void GetReferenceLine(const ReferenceLine &frenet_path,
                        const int host_match_point_index,
                        ReferenceLine &reference_line);

  /**
   * @brief 非首次截取进行平滑轨迹的拼接,
   * 由全局路径转换参考线，即(x,y)->(x,y,heading,kappa)
   * @param routing_path_points
   * @param frenet_path
   */
  void RoutingPathToFrenetPath(const std::vector<MapPoint> &routing_path_points,
                               ReferenceLine *frenet_path);

  ReferenceLine frenet_path_;
  std::vector<MapPoint> pre_points_;
  bool is_first_run_ = false;
  ReferenceLine raw_reference_line_;
  ReferenceLine smoothed_reference_line_;
  ReferencePoint host_project_point_;
  ReferencePoint host_match_point_;
};
}  // namespace ADPlanning
