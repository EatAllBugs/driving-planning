cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

project(EMplanner)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Debug")

# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(config)
add_subdirectory(reference_line)
add_subdirectory(EMPlanner)
add_subdirectory(plot)
add_subdirectory(routing)
add_subdirectory(localization)
add_subdirectory(perception)

include_directories(${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
    "/usr/include/python3.5m"
    "usr/local/include/eigen3/Eigen/"
)

# Create the executable
add_executable(path_plan_test path_plan_test.cpp)

link_directories(
    "usr/local/lib"
    "usr/local/lib64"
    "usr/lib"
    ${PROJECT_BINARY_DIR}/lib
)

message(${PROJECT_BINARY_DIR})

target_include_directories(path_plan_test PUBLIC
    ${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
    "/usr/include/python3.5m"
    SYSTEM ${EIGEN3_INCLUDE_DIR}
)

# link_directories( ${subprojects_SOURCE_DIR}/lib )

# Link the static library from subproject1 using it's alias sub::lib1
# Link the header only library from subproject2 using it's alias sub::lib2
# This will cause the include directories for that target to be added to this project

# <LibaryName>_FOUND
# <LibaryName>_INCLUDE_DIR or <LibaryName>_INCLUDES <LibaryName>_LIBRARY or <LibaryName>_LIBRARIES
find_package(Eigen3 REQUIRED)

# message(${EIGEN3_INCLUDE_DIRS})
# file(GLOB GLOG_LIBRARIES /usr/lib/x86_64-linux-gnu/libglog.so)
find_package(OsqpEigen)

target_link_libraries(path_plan_test PUBLIC
    reference_line_provider
    plot
    routing
    localization
    perception
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

# include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
