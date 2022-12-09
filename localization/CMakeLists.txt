aux_source_directory(. LOCALIZATION_SRC)

add_library(localization ${LOCALIZATION_SRC})

target_include_directories(localization PUBLIC
    ${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
)

target_link_libraries(localization PUBLIC
    EMPlanner
)