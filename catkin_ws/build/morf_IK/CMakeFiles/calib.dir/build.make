# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/leonor/tese/IK_MORF/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leonor/tese/IK_MORF/catkin_ws/build

# Include any dependencies generated for this target.
include morf_IK/CMakeFiles/calib.dir/depend.make

# Include the progress variables for this target.
include morf_IK/CMakeFiles/calib.dir/progress.make

# Include the compile flags for this target's objects.
include morf_IK/CMakeFiles/calib.dir/flags.make

morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o: morf_IK/CMakeFiles/calib.dir/flags.make
morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o: /home/leonor/tese/IK_MORF/catkin_ws/src/morf_IK/src/calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonor/tese/IK_MORF/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o"
	cd /home/leonor/tese/IK_MORF/catkin_ws/build/morf_IK && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calib.dir/src/calibration.cpp.o -c /home/leonor/tese/IK_MORF/catkin_ws/src/morf_IK/src/calibration.cpp

morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calib.dir/src/calibration.cpp.i"
	cd /home/leonor/tese/IK_MORF/catkin_ws/build/morf_IK && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonor/tese/IK_MORF/catkin_ws/src/morf_IK/src/calibration.cpp > CMakeFiles/calib.dir/src/calibration.cpp.i

morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calib.dir/src/calibration.cpp.s"
	cd /home/leonor/tese/IK_MORF/catkin_ws/build/morf_IK && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonor/tese/IK_MORF/catkin_ws/src/morf_IK/src/calibration.cpp -o CMakeFiles/calib.dir/src/calibration.cpp.s

morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.requires:

.PHONY : morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.requires

morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.provides: morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.requires
	$(MAKE) -f morf_IK/CMakeFiles/calib.dir/build.make morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.provides.build
.PHONY : morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.provides

morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.provides.build: morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o


# Object files for target calib
calib_OBJECTS = \
"CMakeFiles/calib.dir/src/calibration.cpp.o"

# External object files for target calib
calib_EXTERNAL_OBJECTS =

/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK/calib: morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o
/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK/calib: morf_IK/CMakeFiles/calib.dir/build.make
/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK/calib: morf_IK/CMakeFiles/calib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonor/tese/IK_MORF/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK/calib"
	cd /home/leonor/tese/IK_MORF/catkin_ws/build/morf_IK && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
morf_IK/CMakeFiles/calib.dir/build: /home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK/calib

.PHONY : morf_IK/CMakeFiles/calib.dir/build

morf_IK/CMakeFiles/calib.dir/requires: morf_IK/CMakeFiles/calib.dir/src/calibration.cpp.o.requires

.PHONY : morf_IK/CMakeFiles/calib.dir/requires

morf_IK/CMakeFiles/calib.dir/clean:
	cd /home/leonor/tese/IK_MORF/catkin_ws/build/morf_IK && $(CMAKE_COMMAND) -P CMakeFiles/calib.dir/cmake_clean.cmake
.PHONY : morf_IK/CMakeFiles/calib.dir/clean

morf_IK/CMakeFiles/calib.dir/depend:
	cd /home/leonor/tese/IK_MORF/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonor/tese/IK_MORF/catkin_ws/src /home/leonor/tese/IK_MORF/catkin_ws/src/morf_IK /home/leonor/tese/IK_MORF/catkin_ws/build /home/leonor/tese/IK_MORF/catkin_ws/build/morf_IK /home/leonor/tese/IK_MORF/catkin_ws/build/morf_IK/CMakeFiles/calib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : morf_IK/CMakeFiles/calib.dir/depend

