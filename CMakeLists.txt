cmake_minimum_required(VERSION 3.15)
project(TrajectorySimulator VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_TESTS "Build tests" OFF)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/cpp)

# Source files
set(MATH_SOURCES
    cpp/math/vector.hpp
    cpp/math/quaternion.hpp
)

set(FRAMES_SOURCES
    cpp/frames/transform.hpp
)

set(DYNAMICS_SOURCES
    cpp/dynamics/rigid_body.hpp
    cpp/dynamics/rigid_body.cpp
)

set(INTEGRATORS_SOURCES
    cpp/integrators/rk4.hpp
    cpp/integrators/rk4.cpp
)

set(MODELS_SOURCES
    cpp/models/aero_model.hpp
    cpp/models/atmosphere.hpp
    cpp/models/thrust.hpp
)

set(API_SOURCES
    cpp/api/simulator_api.hpp
    cpp/api/simulator_api.cpp
)

# Create library
add_library(trajectory_simulator
    ${DYNAMICS_SOURCES}
    ${INTEGRATORS_SOURCES}
    ${API_SOURCES}
)

# Set library properties
set_target_properties(trajectory_simulator PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    POSITION_INDEPENDENT_CODE ON
)

# Python bindings (optional, using pybind11 if available)
find_package(pybind11 QUIET)
if(pybind11_FOUND)
    message(STATUS "pybind11 found - Python bindings will be built")
    
    pybind11_add_module(trajectory_simulator_py
        ${API_SOURCES}
        cpp/api/python_bindings.cpp
    )
    
    target_link_libraries(trajectory_simulator_py PRIVATE trajectory_simulator)
else()
    message(STATUS "pybind11 not found - using ctypes interface only")
    message(STATUS "Install pybind11 for Python bindings: pip install pybind11")
endif()

# Install rules
install(TARGETS trajectory_simulator
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${API_SOURCES}
    DESTINATION include/trajectory_simulator
)

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Platform-specific settings
if(WIN32)
    # Windows-specific settings
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
elseif(UNIX)
    # Unix-specific settings
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
