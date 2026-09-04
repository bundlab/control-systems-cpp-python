cmake_minimum_required(VERSION 3.20)
project(Lab3Sim LANGUAGES CXX)

add_executable(lab3_sim main.cpp pid_controller.hpp)

if(TARGET project_warnings)
    target_link_libraries(lab3_sim PRIVATE project_warnings)
else()
    target_compile_options(lab3_sim PRIVATE
        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wpedantic -O3>
        $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic -O3>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2>
    )
endif()
