add_executable(tests test.cpp)
target_link_libraries(tests PUBLIC CORA)
message(STATUS "Building test command-line executable in directory ${EXECUTABLE_OUTPUT_PATH}\n")
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
