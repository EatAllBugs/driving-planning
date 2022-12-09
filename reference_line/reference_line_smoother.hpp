/*
此类使用Apollo的discrete_points_smoother的FEM_POS_DEVIATION_SMOOTHING
*/
#include "config/reference_line_smoother_config.hpp"
#include "reference_line/reference_line.hpp"
namespace ADPlanning {
class ReferenceLineSmoother {
 public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig &config);
  ~ReferenceLineSmoother() = default;

  // 1.平滑

  void Smooth(const ReferenceLine &raw_reference_line,
              ReferenceLine &smoothed_reference_line);

 private:
  ReferenceLineSmootherConfig config_;
  // 对2d曲线进行平滑处理
  bool DiscretePointsSmooth(
      const std::vector<std::pair<double, double>> &raw_point2d,
      std::vector<std::pair<double, double>> *ptr_smoothed_point2d);
};
}  // namespace ADPlanning