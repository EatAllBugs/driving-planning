/*
  此类的功能主要用于构造参考线类型的变量
*/

#pragma once

#include <vector>

#include "common/pnc_point.hpp"
namespace ADPlanning {
class ReferenceLine {
 public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;
  void set_reference_points(std::vector<ReferencePoint> reference_points);
  void set_match_point_index(int index);
  void set_host_match_point(ReferencePoint host_match_point);
  void set_host_project_point(ReferencePoint host_project_point);

  const std::vector<ReferencePoint> reference_points() const;
  const int match_point_index() const;
  const ReferencePoint host_project_point() const;
  const ReferencePoint host_match_point() const;

 private:
  std::vector<ReferencePoint> reference_points_;  // 参考点类
  int match_point_index_;                         // 自车匹配点索引
  ReferencePoint host_project_point_;
  ReferencePoint host_match_point_;
};
}  // namespace ADPlanning