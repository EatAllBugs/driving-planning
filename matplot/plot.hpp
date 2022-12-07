#include <map>
#include <vector>

#include "EMPlanner/trajectory.hpp"
#include "matplot/matplotlibcpp.h"
#include "reference_line/reference_line.hpp"
#include "routing/routing_path.hpp"
using namespace std;
namespace plt = matplotlibcpp;

namespace ADPlanning {
class Plot {
 public:
  Plot();
  ~Plot() = default;
  void PlotRoutingPath(std::vector<MapPoint> routing_path_points,
                       const std::string &color);
  void PlotReferenceLine(ReferenceLine reference_line,
                         const std::string &color);
  void PlotTrajetory(Trajectory trajectory, const std::string &color);
};
}  // namespace ADPlanning