aux_source_directory(. REFERENCELINE_SRC)

add_library(reference_line_provider ${REFERENCELINE_SRC})

find_package(Eigen3)
find_package(OsqpEigen)

target_include_directories(reference_line_provider PUBLIC 
                            ${PROJECT_SOURCE_DIR}
                            "usr/include"
                            "usr/local/include"
)

target_link_libraries(reference_line_provider PUBLIC
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)