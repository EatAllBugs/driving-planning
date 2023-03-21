/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */

#include "config/reference_line_smoother_config.hpp"
#include "reference_line/reference_line.hpp"
namespace ADPlanning {
class ReferenceLineSmoother {
 public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig &config);
  ~ReferenceLineSmoother() = default;
  /**
   * @brief
   * @param raw_reference_line
   * @param smoothed_reference_line
   */
  void Smooth(const ReferenceLine &raw_reference_line,
              ReferenceLine &smoothed_reference_line);

 private:
  ReferenceLineSmootherConfig config_;
  /**
   * @brief
   * @param raw_point2d
   * @param ptr_smoothed_point2d
   * @return bool
   */
  bool DiscretePointsSmooth(
      const std::vector<std::pair<double, double>> &raw_point2d,
      std::vector<std::pair<double, double>> *ptr_smoothed_point2d);
};
}  // namespace ADPlanning