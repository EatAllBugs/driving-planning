aux_source_directory(. EMPLANNER_SRC)

add_library(EMPlanner ${EMPLANNER_SRC})

target_include_directories(EMPlanner PUBLIC
    ${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
)