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

#remove flag
#string(REPLACE "-FLAG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

#add flag
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -FLAG")

#error out and see flags
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -HAHAHA")


set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
#string(REPLACE "-O3" "-Os" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

include_directories("include/eigen-3.4.0")

set(SFML_DIR "include/SFML-2.6.1/lib/cmake/SFML")

find_package(SFML COMPONENTS graphics window REQUIRED)

target_link_libraries(MINREP PUBLIC sfml-graphics sfml-window)