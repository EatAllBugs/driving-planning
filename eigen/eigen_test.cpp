/**
 * Copyright (C) 2023 by EatAllBugs Limited. All rights reserved.
 * EatAllBugs <lysxx717@gmail.com>
 */

#include <iostream>

#include "Eigen/Dense"

int main() {
  Eigen::Matrix3d Matrixcomp;
  Matrixcomp << 1, 1, 1, 2, 2, 2, 3, 3, 3;
  std::cout << Matrixcomp << std::endl;
}