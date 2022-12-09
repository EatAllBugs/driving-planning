set(Python3_INCLUDE_DIRS "/usr/include/python3.5m")
set(Python3_LIBRARIES "/usr/lib/python3.5/config-3.5m-x86_64-linux-gnu/libpython3.5m.so")
set(PYTHON_EXECUTABLE "/usr/bin/python3.5")

# set(Numpy_INCLUDE_DIRS "/usr/include/python3.6m/numpy")
# set(Numpy_LIBRARIES "/usr/lib/x86_64-linux-gnu/libboost_numpy.so")

# Library target
add_library(matplotlib_cpp INTERFACE)

target_include_directories(matplotlib_cpp INTERFACE
  ${PROJECT_SOURCE_DIR}
  "usr/include"
  "usr/local/include"
  ${Python3_INCLUDE_DIRS}
)

target_link_libraries(matplotlib_cpp INTERFACE
  ${Python3_LIBRARIES}
)

target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_17
)

add_library(plot plot.cc)
find_package(Eigen3)
target_include_directories(plot PUBLIC
  ${PROJECT_SOURCE_DIR}
  "usr/include"
  "usr/local/include"
  "/usr/include/python3.6m"
)

target_link_libraries(plot PUBLIC
  matplotlib_cpp
  Eigen3::Eigen
  routing
  reference_line_provider
  EMPlanner
  localization
  perception
)