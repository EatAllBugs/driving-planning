aux_source_directory(. ROUTING_SRC)

add_library(routing ${ROUTING_SRC})

target_include_directories(routing PUBLIC
    ${PROJECT_SOURCE_DIR}
    "usr/include"
    "usr/local/include"
)

# target_link_libraries(routing PUBLIC)