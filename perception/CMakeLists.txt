aux_source_directory(. PERCETION_SRC)

add_library(perception ${PERCETION_SRC})

target_include_directories(perception PUBLIC
    ${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
)

# target_link_libraries(perception PUBLIC              )