set(Python3_INCLUDE_DIRS "/usr/include/python3.6m")
set(Python3_LIBRARIES "/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so")
set(PYTHON_EXECUTABLE "/usr/bin/python3.6")

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
  cxx_std_14
)
