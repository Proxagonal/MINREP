# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/cmake-build-valkyrie

# Include any dependencies generated for this target.
include CMakeFiles/MINREP.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/MINREP.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/MINREP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MINREP.dir/flags.make

CMakeFiles/MINREP.dir/main.cpp.o: CMakeFiles/MINREP.dir/flags.make
CMakeFiles/MINREP.dir/main.cpp.o: ../main.cpp
CMakeFiles/MINREP.dir/main.cpp.o: CMakeFiles/MINREP.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/cmake-build-valkyrie/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MINREP.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MINREP.dir/main.cpp.o -MF CMakeFiles/MINREP.dir/main.cpp.o.d -o CMakeFiles/MINREP.dir/main.cpp.o -c /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/main.cpp

CMakeFiles/MINREP.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MINREP.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/main.cpp > CMakeFiles/MINREP.dir/main.cpp.i

CMakeFiles/MINREP.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MINREP.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/main.cpp -o CMakeFiles/MINREP.dir/main.cpp.s

# Object files for target MINREP
MINREP_OBJECTS = \
"CMakeFiles/MINREP.dir/main.cpp.o"

# External object files for target MINREP
MINREP_EXTERNAL_OBJECTS =

MINREP: CMakeFiles/MINREP.dir/main.cpp.o
MINREP: CMakeFiles/MINREP.dir/build.make
MINREP: ../include/SFML-2.6.0/bin/sfml-graphics-d-2.dll
MINREP: ../include/SFML-2.6.0/bin/sfml-window-d-2.dll
MINREP: ../include/SFML-2.6.0/bin/sfml-system-d-2.dll
MINREP: CMakeFiles/MINREP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/cmake-build-valkyrie/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable MINREP"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MINREP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MINREP.dir/build: MINREP
.PHONY : CMakeFiles/MINREP.dir/build

CMakeFiles/MINREP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MINREP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MINREP.dir/clean

CMakeFiles/MINREP.dir/depend:
	cd /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/cmake-build-valkyrie && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/cmake-build-valkyrie /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/cmake-build-valkyrie /mnt/c/Users/eitan/Desktop/NBPSPACE/MINREP/cmake-build-valkyrie/CMakeFiles/MINREP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MINREP.dir/depend

