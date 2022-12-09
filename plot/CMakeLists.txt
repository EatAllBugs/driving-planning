add_subdirectory(matplot)

add_library(plot plot.cpp)
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