# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hhy/drone/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hhy/drone/build

# Include any dependencies generated for this target.
include FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/depend.make

# Include the progress variables for this target.
include FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/progress.make

# Include the compile flags for this target's objects.
include FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/flags.make

FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o: FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/flags.make
FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o: /home/hhy/drone/src/FUEL/uav_simulator/so3_control/src/SO3Control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hhy/drone/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o"
	cd /home/hhy/drone/build/FUEL/uav_simulator/so3_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o -c /home/hhy/drone/src/FUEL/uav_simulator/so3_control/src/SO3Control.cpp

FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/src/SO3Control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SO3Control.dir/src/SO3Control.cpp.i"
	cd /home/hhy/drone/build/FUEL/uav_simulator/so3_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hhy/drone/src/FUEL/uav_simulator/so3_control/src/SO3Control.cpp > CMakeFiles/SO3Control.dir/src/SO3Control.cpp.i

FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/src/SO3Control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SO3Control.dir/src/SO3Control.cpp.s"
	cd /home/hhy/drone/build/FUEL/uav_simulator/so3_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hhy/drone/src/FUEL/uav_simulator/so3_control/src/SO3Control.cpp -o CMakeFiles/SO3Control.dir/src/SO3Control.cpp.s

# Object files for target SO3Control
SO3Control_OBJECTS = \
"CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o"

# External object files for target SO3Control
SO3Control_EXTERNAL_OBJECTS =

/home/hhy/drone/devel/lib/libSO3Control.so: FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o
/home/hhy/drone/devel/lib/libSO3Control.so: FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/build.make
/home/hhy/drone/devel/lib/libSO3Control.so: FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hhy/drone/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/hhy/drone/devel/lib/libSO3Control.so"
	cd /home/hhy/drone/build/FUEL/uav_simulator/so3_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SO3Control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/build: /home/hhy/drone/devel/lib/libSO3Control.so

.PHONY : FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/build

FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/clean:
	cd /home/hhy/drone/build/FUEL/uav_simulator/so3_control && $(CMAKE_COMMAND) -P CMakeFiles/SO3Control.dir/cmake_clean.cmake
.PHONY : FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/clean

FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/depend:
	cd /home/hhy/drone/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hhy/drone/src /home/hhy/drone/src/FUEL/uav_simulator/so3_control /home/hhy/drone/build /home/hhy/drone/build/FUEL/uav_simulator/so3_control /home/hhy/drone/build/FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : FUEL/uav_simulator/so3_control/CMakeFiles/SO3Control.dir/depend

