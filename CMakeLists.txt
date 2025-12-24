cmake_minimum_required(VERSION 3.9)
project(MINREP)

set(CMAKE_CXX_STANDARD 23)

add_executable(MINREP
        main.cpp
        #TEST_OE.cpp
        Consts.h
        SEE.h
        SOWSKIP.h
        HALT.h
        OrbitalElements.h
        #Solver.h
        #Visualizer.h
        #Quantities.h
        #Bodyfold.h
        #Kepler.h
)

#remove flag
#string(REPLACE "-FLAG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

#add flag
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -FLAG")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fsanitize=address -g") #HEYO DEBUG

# TO SEE pref:  -g -fno-omit-frame-pointer -fno-inline

#string(REPLACE "-O3" "-Os" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

#error out and see flags
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -HAHAHA")


include_directories("include/eigen-3.4.0")


find_package(SFML COMPONENTS graphics window REQUIRED)

target_link_libraries(MINREP PUBLIC sfml-graphics sfml-window)