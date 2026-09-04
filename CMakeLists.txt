cmake_minimum_required(VERSION 3.20)
project(ControlSystemsCppPython LANGUAGES CXX)

# Force C++20 standard globally
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for language servers (clangd, ccls, vscode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Configure default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Strict compiler options for code quality
add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow -O3>
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow -O3>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2>
)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# Subdirectories for labs
# Add new lab subdirectories here as they are developed
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/labs/lab_1")
    add_subdirectory(labs/lab_1)
endif()

# Future subdirectories template:
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/labs/lab_2")
    add_subdirectory(labs/lab_2)
endif()