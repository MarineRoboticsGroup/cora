# SE-Sync command-line driver
add_executable(cora_example main.cpp)
target_link_libraries(cora_example CORA)
if(${ENABLE_PROFILING})
target_compile_definitions(cora_example PRIVATE GPERFTOOLS)
target_link_libraries(cora_example profiler)
endif()

set(EXAMPLE_PYFG_FILES
"factor_graph.pyfg"
)

message(STATUS "Building main cora_example command-line executable in directory ${EXECUTABLE_OUTPUT_PATH}\n")

# SE-Sync visualizer
if(${ENABLE_VISUALIZATION})
  message(WARNING "Visualization is not implemented yet")
endif()

## Copy example PyFG files to build directory
foreach(PYFG_FILE ${EXAMPLE_PYFG_FILES})
    configure_file(${PYFG_FILE} ${EXECUTABLE_OUTPUT_PATH}/example_data/${PYFG_FILE} COPYONLY)
endforeach()
