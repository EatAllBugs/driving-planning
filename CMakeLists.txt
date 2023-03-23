cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

project(EMplanner)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Debug")

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
    "/usr/include/python3.6m"
    "usr/local/include/eigen3/Eigen/"
)

# Create the executable
add_executable(path_plan_test path_plan_test.cpp)
add_executable(main main.cpp)
add_executable(reference_line_test reference_line_test.cpp)

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
    "/usr/include/python3.6m"
    SYSTEM ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(main PUBLIC
    ${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
    "/usr/include/python3.6m"
    SYSTEM ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(reference_line_test PUBLIC
    ${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
    "/usr/include/python3.6m"
    SYSTEM ${EIGEN3_INCLUDE_DIR}
)

find_package(Eigen3 REQUIRED)

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

target_link_libraries(main PUBLIC
    reference_line_provider
    plot
    routing
    localization
    perception
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

target_link_libraries(reference_line_test PUBLIC
    reference_line_provider
    plot
    routing
    localization
    perception
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)
