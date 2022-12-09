
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
    const std::vector<MapPoint> routing_path_point) {
  routing_path_points_.assign(routing_path_point.begin(),
                              routing_path_point.end());
}

// 从csv文件中初始化routing_path
std::vector<MapPoint> RoutingPath::GetRoutingPathFromCSV() {
  // 读取csv文件

  std::ifstream fp(
      ".\\file\\routing_path.csv");  // 定义声明一个ifstream对象，指定文件路径

  if (!fp.is_open()) {
    std::cout << "Error: opening file fail" << std::endl;
    std::exit(1);
  }
  std::string line;

  std::istringstream sin;  // 将整行字符串line读入到字符串istringstream中
  std::vector<std::string> words;  // 声明一个字符串向量
  std::string word;

  std::getline(fp, line);  // 读取标题行
  std::vector<MapPoint> path;
  while (std::getline(fp, line))  // 从文件fp读取一行数据至line
  {
    sin.clear();
    sin.str(line);
    words.clear();
    int col = 1;
    MapPoint point;
    while (std::getline(
        sin, word,
        ','))  // 将字符串流sin中的字符读到field字符串中，以逗号为分隔符
    {
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