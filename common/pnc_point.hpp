/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */
#pragma once

namespace ADPlanning {

class MapPoint {
 public:
  double x;
  double y;
};

class ReferencePoint : public MapPoint {
 public:
  double index;
  double heading;
  double kappa;
  double dkappa;
};

class TrajectoryPoint : public ReferencePoint {
 public:
  double v;
  double vx;
  double vy;
  double a;
  double ax;
  double ay;
  double t;
};

class ObstacleInfo : public TrajectoryPoint {
 public:
  int id;
  double init_x;
  double init_y;
  double init_heading;
  double v;
  double heading;
};

class LocalizationInfo : public TrajectoryPoint {
 public:
};

class SLPoint {
 public:
  int index;
  double s;
  double ds_dt;
  double dds_dt;
  double l;
  double dl_dt;
  double ddl_dt;
  double dddl_dt;
  double dl_ds;
  double ddl_ds;
  double dddl_ds;

  double cost2start;
  int pre_mincost_row;
};

class STPoint {
 public:
  double t;
  double s;
  double ds_dt;
  double dds_dt;

  double cost2start;
  int pre_mincost_row;
};

class STLine {
 public:
  STPoint left_point;
  STPoint right_point;
};

}  // namespace ADPlanning