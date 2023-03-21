/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#pragma once

#include <cmath>
#include <vector>

#include "common/pnc_point.hpp"
namespace ADPlanning {
class PerceptionObstacle {
 public:
  PerceptionObstacle();
  ~PerceptionObstacle() = default;
  /**
   * @brief
   * @return const std::vector<ObstacleInfo>
   */
  const std::vector<ObstacleInfo> static_obstacles() const;

  /**
   * @brief
   * @return const std::vector<ObstacleInfo>
   */
  const std::vector<ObstacleInfo> dynamic_obstacles() const;

  /**
   * @brief 对障碍物列表的状态进行更新
   * @param time
   * @param localization_info
   */
  void UpdateObstacleInfo(const int time,
                          const LocalizationInfo &localization_info);

  /**
   * @brief
   * @param xy_virtual_obstacles
   */
  void UpdateVirtualObstacle(
      const std::vector<ReferencePoint> &xy_virtual_obstacles);

  /**
   * @brief
   * @param id
   * @param init_x
   * @param init_y
   * @param init_heading
   * @param init_v
   */
  void AddStaticObstacle(int id, double init_x, double init_y,
                         double init_heading, double init_v);

  /**
   * @brief
   * @param id
   * @param init_x
   * @param init_y
   * @param init_heading
   * @param init_v
   */
  void AddDynamicObstacle(const int id, const double init_x,
                          const double init_y, const double init_heading,
                          const double init_v);

  /**
   * @brief 根据车辆的位置输出障碍物信息
   * @param localization_info
   */
  void FilterAndOutObstacleInfo(const LocalizationInfo &localization_info);

 private:
  std::vector<ObstacleInfo> static_obstacles_;
  std::vector<ObstacleInfo> dynamic_obstacles_;
};
}  // namespace ADPlanning