/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#include "routing/routing_path.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "common/pnc_point.hpp"
namespace ADPlanning {
const std::vector<MapPoint> RoutingPath::routing_path_points() const {
  return routing_path_points_;
}

void RoutingPath::set_routing_path(
    const std::vector<MapPoint> &routing_path_point) {
  routing_path_points_.assign(routing_path_point.begin(),
                              routing_path_point.end());
}

std::vector<MapPoint> RoutingPath::GetRoutingPathFromCSV() {
  std::string csv_path = ".\\file\\routing_path.csv";

  std::ifstream fp(csv_path);

  if (!fp.is_open()) {
    std::cout << "Error: opening file fail" << std::endl;
    std::exit(1);
  }
  std::string line;

  std::istringstream sin;
  std::vector<std::string> words;
  std::string word;

  std::getline(fp, line);
  std::vector<MapPoint> path;
  while (std::getline(fp, line)) {
    sin.clear();
    sin.str(line);
    words.clear();
    int col = 1;
    MapPoint point;
    while (std::getline(sin, word, ',')) {
      if (col == 1) {
        point.x = stod(word);
      } else {
        point.y = stod(word);
      }
      std::cout << word;
      col++;
    }
    path.push_back(point);
  }
  fp.close();

  return path;
}

void RoutingPath::CreatePath() {
  routing_path_points_.clear();
  MapPoint point;
  point.x = 0;
  point.y = 0;
  routing_path_points_.push_back(point);
  double x = 0, y = 0;
  // 第1段
  for (int i = 0; i < 60; i++) {
    x = x + 1;
    y = y + 0;
    point.x = x;
    point.y = y;
    routing_path_points_.push_back(point);
  }

  // 第2段
  for (int i = 0; i < 20; i++) {
    x = x;
    y = y + 1;
    point.x = x;
    point.y = y;
    routing_path_points_.push_back(point);
  }
  // 第3段
  for (int i = 0; i < 50; i++) {
    x = x + 1;
    y = y;
    point.x = x;
    point.y = y;
    routing_path_points_.push_back(point);
  }

  // 第4段
  for (int i = 0; i < 20; i++) {
    x = x;
    y = y - 1;
    point.x = x;
    point.y = y;
    routing_path_points_.push_back(point);
  }

  // 第5段
  for (int i = 0; i < 500; i++) {
    x = x + 1;
    y = y;
    point.x = x;
    point.y = y;
    routing_path_points_.push_back(point);
  }
}
}  // namespace ADPlanning