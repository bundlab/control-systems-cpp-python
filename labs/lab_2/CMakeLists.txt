cmake_minimum_required(VERSION 3.20)
project(Lab2Sim LANGUAGES CXX)

add_executable(lab2_sim main.cpp discrete_system.hpp)

if(TARGET project_warnings)
    target_link_libraries(lab2_sim PRIVATE project_warnings)
else()
    target_compile_options(lab2_sim PRIVATE
        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wpedantic -O3>
        $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic -O3>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2>
    )
endif()
