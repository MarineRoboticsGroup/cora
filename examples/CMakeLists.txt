add_executable(cora_example main.cpp)
target_link_libraries(cora_example CORA)

add_executable(example_solve_expand example_solve_expand.cpp)
target_link_libraries(example_solve_expand CORA)

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
  target_include_directories(paper_experiments PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  if(${ENABLE_PROFILING})
    target_compile_definitions(paper_experiments PRIVATE GPERFTOOLS)
    target_link_libraries(paper_experiments profiler)
  endif()
endif()

set(EXAMPLE_PYFG_FILES
"config.json"
"data/factor_graph_small.pyfg"
"data/mrclam/range_and_rpm/mrclam2/mrclam2.pyfg"
"data/mrclam/range_and_rpm/mrclam3a/mrclam3a.pyfg"
"data/mrclam/range_and_rpm/mrclam3b/mrclam3b.pyfg"
"data/mrclam/range_and_rpm/mrclam4/mrclam4.pyfg"
"data/mrclam/range_and_rpm/mrclam5a/mrclam5a.pyfg"
"data/mrclam/range_and_rpm/mrclam5b/mrclam5b.pyfg"
"data/mrclam/range_and_rpm/mrclam5c/mrclam5c.pyfg"
"data/mrclam/range_and_rpm/mrclam6/mrclam6.pyfg"
"data/mrclam/range_and_rpm/mrclam7/mrclam7.pyfg"
"data/plaza1.pyfg"
"data/plaza2.pyfg"
"data/single_drone.pyfg"
"data/tiers.pyfg"
)

message(STATUS "Building main cora_example command-line executable in directory ${EXECUTABLE_OUTPUT_PATH}\n")

## Copy example PyFG files to build directory
foreach(PYFG_FILE ${EXAMPLE_PYFG_FILES})
    configure_file(${PYFG_FILE} ${EXECUTABLE_OUTPUT_PATH}/${PYFG_FILE} COPYONLY)
endforeach()
