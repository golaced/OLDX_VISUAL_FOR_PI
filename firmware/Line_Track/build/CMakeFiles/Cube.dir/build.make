# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/pi/QT/Cube_Tracker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/QT/Cube_Tracker/build

# Include any dependencies generated for this target.
include CMakeFiles/Cube.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Cube.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Cube.dir/flags.make

CMakeFiles/Cube.dir/src/main.cpp.o: CMakeFiles/Cube.dir/flags.make
CMakeFiles/Cube.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/QT/Cube_Tracker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Cube.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Cube.dir/src/main.cpp.o -c /home/pi/QT/Cube_Tracker/src/main.cpp

CMakeFiles/Cube.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Cube.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/QT/Cube_Tracker/src/main.cpp > CMakeFiles/Cube.dir/src/main.cpp.i

CMakeFiles/Cube.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Cube.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/QT/Cube_Tracker/src/main.cpp -o CMakeFiles/Cube.dir/src/main.cpp.s

CMakeFiles/Cube.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/Cube.dir/src/main.cpp.o.requires

CMakeFiles/Cube.dir/src/main.cpp.o.provides: CMakeFiles/Cube.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Cube.dir/build.make CMakeFiles/Cube.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/Cube.dir/src/main.cpp.o.provides

CMakeFiles/Cube.dir/src/main.cpp.o.provides.build: CMakeFiles/Cube.dir/src/main.cpp.o


# Object files for target Cube
Cube_OBJECTS = \
"CMakeFiles/Cube.dir/src/main.cpp.o"

# External object files for target Cube
Cube_EXTERNAL_OBJECTS =

../bin/Cube: CMakeFiles/Cube.dir/src/main.cpp.o
../bin/Cube: CMakeFiles/Cube.dir/build.make
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_ts.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_gpu.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_contrib.so.2.4.9
../bin/Cube: /usr/local/lib/libaruco.so.3.0.6
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.9
../bin/Cube: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.9
../bin/Cube: CMakeFiles/Cube.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/QT/Cube_Tracker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/Cube"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Cube.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Cube.dir/build: ../bin/Cube

.PHONY : CMakeFiles/Cube.dir/build

CMakeFiles/Cube.dir/requires: CMakeFiles/Cube.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/Cube.dir/requires

CMakeFiles/Cube.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Cube.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Cube.dir/clean

CMakeFiles/Cube.dir/depend:
	cd /home/pi/QT/Cube_Tracker/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/QT/Cube_Tracker /home/pi/QT/Cube_Tracker /home/pi/QT/Cube_Tracker/build /home/pi/QT/Cube_Tracker/build /home/pi/QT/Cube_Tracker/build/CMakeFiles/Cube.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Cube.dir/depend

