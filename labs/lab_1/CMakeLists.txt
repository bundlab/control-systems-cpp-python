cmake_minimum_required(VERSION 3.20)
project(Lab1Sim LANGUAGES CXX)

add_executable(lab1_sim main.cpp state_space.hpp)

if(TARGET project_warnings)
    target_link_libraries(lab1_sim PRIVATE project_warnings)
else()
    target_compile_options(lab1_sim PRIVATE
        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wpedantic -O3>
        $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic -O3>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2>
    )
endif()