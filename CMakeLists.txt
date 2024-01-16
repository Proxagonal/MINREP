cmake_minimum_required(VERSION 3.9)
project(MINREP)

set(CMAKE_CXX_STANDARD 23)

add_executable(MINREP
        main.cpp
        Solver.h
        Visualizer.h
        Quantities.h
        Runner.h
        Bodyfold.h
)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

include_directories("include/eigen-3.4.0")

set(SFML_DIR "include/SFML-2.6.1/lib/cmake/SFML")

find_package(SFML COMPONENTS graphics window REQUIRED)

target_link_libraries(MINREP PUBLIC sfml-graphics sfml-window)