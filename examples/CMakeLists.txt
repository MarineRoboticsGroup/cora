# SE-Sync command-line driver
add_executable(cora_example main.cpp)
target_link_libraries(cora_example CORA)

if(${ENABLE_VISUALIZATION})
  message(STATUS "Building CORAVis example")
  add_executable(cora_vis cora_vis_test.cpp)
  target_link_libraries(cora_vis CORA)

  set(DATA_FILE "factor_graph_small.pyfg")
  configure_file(${DATA_FILE} ${EXECUTABLE_OUTPUT_PATH}/${DATA_FILE} COPYONLY)

endif()

if(${ENABLE_PROFILING})
target_compile_definitions(cora_example PRIVATE GPERFTOOLS)
target_link_libraries(cora_example profiler)
endif()


message(STATUS "Building main cora_example command-line executable in directory ${EXECUTABLE_OUTPUT_PATH}\n")
