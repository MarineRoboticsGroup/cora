if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
    message(STATUS "Setting build type to 'Debug' as none was specified.")
endif()

add_executable(tests
    test.cpp
    test_parse_pyfg.cpp
    test_construct_problem.cpp
    test_geometry.cpp
    test_cora.cpp
    test_optimizer_helpers.cpp
)

message(STATUS "Building test command-line executable in directory ${EXECUTABLE_OUTPUT_PATH}\n")
target_link_libraries(tests PUBLIC CORA)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

## The basic data that we expect to be in the data directory
set(DATA_DIR "data")
set(EXPECTED_FILES
"factor_graph.pyfg"
"DataMatrix.mm"
# relative pose data
"T.mm"
"Apose.mm"
"OmegaPose.mm"
"RotConLaplacian.mm"
# range data
"Arange.mm"
"OmegaRange.mm"
"RangeDistances.mm"
# candidate initializations
"X_gt.mm"
"X_odom.mm"
"X_rand_dim2.mm"
# info for testing the optimization helper functions
"expected_egrad.mm"
"expected_rgrad.mm"
"hessProd.mm"
"rand_dX.mm"
)

## For each problem type in our tests, we expect these directories
set(PROBLEM_DIRS "")
foreach(PROBLEM_TYPE "small_ra_slam_problem" "single_range" "single_rpm")
    list(APPEND PROBLEM_DIRS "${DATA_DIR}/${PROBLEM_TYPE}")
endforeach()

## Get all of the different data files we expect
set(DATA_FILES "")
foreach(PROBLEM_DIR ${PROBLEM_DIRS})
foreach(EXPECTED_FILE ${EXPECTED_FILES})
    list(APPEND DATA_FILES "${PROBLEM_DIR}/${EXPECTED_FILE}")
endforeach()
endforeach()

## Copy the data files to the build directory
foreach(DATA_FILE ${DATA_FILES})
    configure_file(${DATA_FILE} ${EXECUTABLE_OUTPUT_PATH}/${DATA_FILE} COPYONLY)
endforeach()
