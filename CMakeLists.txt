cmake_minimum_required(VERSION 3.9)
project(MINREP)

set(CMAKE_CXX_STANDARD 23)

add_executable(MINREP
        main.cpp
        Solver.h
        Body.h
        Visualizer.h
        Quantities.h
        Runner.h
)

add_compile_options(-O3)

include_directories("include/eigen-3.4.0")

find_package(SFML COMPONENTS graphics window REQUIRED)

target_link_libraries(MINREP PUBLIC sfml-graphics sfml-window)