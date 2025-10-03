add_subdirectory(${CMAKE_SOURCE_DIR}/libs/pybind11 ${CMAKE_BINARY_DIR}/pybind11)

pybind11_add_module(cora SHARED
    cora_bindings.cpp
)

target_link_libraries(cora PRIVATE CORA)