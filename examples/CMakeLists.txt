# SE-Sync command-line driver
add_executable(cora_example main.cpp)
target_link_libraries(cora_example CORA)

if(${ENABLE_VISUALIZATION})
  message(STATUS "Building CORAVis example")
  add_executable(cora_vis cora_vis_test.cpp)
  target_link_libraries(cora_vis CORA)
endif()

if(${ENABLE_PROFILING})
target_compile_definitions(cora_example PRIVATE GPERFTOOLS)
target_link_libraries(cora_example profiler)
endif()

if(${PERFORM_EXPERIMENTS})
message(STATUS "Building paper_experiments executable")
add_executable(paper_experiments paper_experiments.cpp)
target_link_libraries(paper_experiments CORA)
endif()

set(EXAMPLE_PYFG_FILES
"data/factor_graph_small.pyfg"
"data/marine_two_robots.pyfg"
"data/plaza1.pyfg"
"data/plaza2.pyfg"
"data/single_drone.pyfg"
"data/tiers.pyfg"
)

message(STATUS "Building main cora_example command-line executable in directory ${EXECUTABLE_OUTPUT_PATH}\n")

## Copy example PyFG files to build directory
foreach(PYFG_FILE ${EXAMPLE_PYFG_FILES})
    configure_file(${PYFG_FILE} ${EXECUTABLE_OUTPUT_PATH}/example_data/${PYFG_FILE} COPYONLY)
endforeach()
