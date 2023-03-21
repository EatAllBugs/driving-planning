/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#pragma once

#include <vector>

#include "common/pnc_point.hpp"
namespace ADPlanning {
class ReferenceLine {
 public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;
  /**
   * @brief Set the reference points object
   * @param reference_points
   */
  void set_reference_points(
      const std::vector<ReferencePoint>& reference_points);

  /**
   * @brief Set the match point index object
   * @param index
   */
  void set_match_point_index(const int index);

  /**
   * @brief Set the host match point object
   * @param host_match_point
   */
  void set_host_match_point(const ReferencePoint& host_match_point);

  /**
   * @brief Set the host project point object
   * @param host_project_point
   */
  void set_host_project_point(const ReferencePoint& host_project_point);

  /**
   * @brief
   * @return const std::vector<ReferencePoint>
   */
  const std::vector<ReferencePoint> reference_points() const;

  /**
   * @brief
   * @return const int
   */
  const int match_point_index() const;
  const ReferencePoint host_project_point() const;
  const ReferencePoint host_match_point() const;

 private:
  std::vector<ReferencePoint> reference_points_;
  int match_point_index_;
  ReferencePoint host_project_point_;
  ReferencePoint host_match_point_;
};
}  // namespace ADPlanning