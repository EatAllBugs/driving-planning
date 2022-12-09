#include "reference_line.hpp"
namespace ADPlanning {
void ReferenceLine::set_reference_points(
    std::vector<ReferencePoint> reference_points) {
  reference_points_.assign(reference_points.begin(), reference_points.end());
}

const std::vector<ReferencePoint> ReferenceLine::reference_points() const {
  return reference_points_;
}

void ReferenceLine::set_match_point_index(int index) {
  match_point_index_ = index;
}

void ReferenceLine::set_host_project_point(ReferencePoint host_project_point) {
  host_project_point_ = host_project_point;
}
void ReferenceLine::set_host_match_point(ReferencePoint host_match_point) {
  host_match_point_ = host_match_point;
}

const int ReferenceLine::match_point_index() const {
  return match_point_index_;
}

const ReferencePoint ReferenceLine::host_project_point() const {
  return host_project_point_;
}
const ReferencePoint ReferenceLine::host_match_point() const {
  return host_match_point_;
}
}  // namespace ADPlanning