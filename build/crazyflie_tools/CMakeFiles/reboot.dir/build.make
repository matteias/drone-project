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
CMAKE_SOURCE_DIR = /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_tools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pascal/dd2419_ws/build/crazyflie_tools

# Include any dependencies generated for this target.
include CMakeFiles/reboot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/reboot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/reboot.dir/flags.make

CMakeFiles/reboot.dir/src/reboot.cpp.o: CMakeFiles/reboot.dir/flags.make
CMakeFiles/reboot.dir/src/reboot.cpp.o: /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_tools/src/reboot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pascal/dd2419_ws/build/crazyflie_tools/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/reboot.dir/src/reboot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/reboot.dir/src/reboot.cpp.o -c /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_tools/src/reboot.cpp

CMakeFiles/reboot.dir/src/reboot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/reboot.dir/src/reboot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_tools/src/reboot.cpp > CMakeFiles/reboot.dir/src/reboot.cpp.i

CMakeFiles/reboot.dir/src/reboot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/reboot.dir/src/reboot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_tools/src/reboot.cpp -o CMakeFiles/reboot.dir/src/reboot.cpp.s

CMakeFiles/reboot.dir/src/reboot.cpp.o.requires:

.PHONY : CMakeFiles/reboot.dir/src/reboot.cpp.o.requires

CMakeFiles/reboot.dir/src/reboot.cpp.o.provides: CMakeFiles/reboot.dir/src/reboot.cpp.o.requires
	$(MAKE) -f CMakeFiles/reboot.dir/build.make CMakeFiles/reboot.dir/src/reboot.cpp.o.provides.build
.PHONY : CMakeFiles/reboot.dir/src/reboot.cpp.o.provides

CMakeFiles/reboot.dir/src/reboot.cpp.o.provides.build: CMakeFiles/reboot.dir/src/reboot.cpp.o


# Object files for target reboot
reboot_OBJECTS = \
"CMakeFiles/reboot.dir/src/reboot.cpp.o"

# External object files for target reboot
reboot_EXTERNAL_OBJECTS =

/home/pascal/dd2419_ws/devel/.private/crazyflie_tools/lib/crazyflie_tools/reboot: CMakeFiles/reboot.dir/src/reboot.cpp.o
/home/pascal/dd2419_ws/devel/.private/crazyflie_tools/lib/crazyflie_tools/reboot: CMakeFiles/reboot.dir/build.make
/home/pascal/dd2419_ws/devel/.private/crazyflie_tools/lib/crazyflie_tools/reboot: /home/pascal/dd2419_ws/devel/.private/crazyflie_cpp/lib/libcrazyflie_cpp.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_tools/lib/crazyflie_tools/reboot: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_tools/lib/crazyflie_tools/reboot: CMakeFiles/reboot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pascal/dd2419_ws/build/crazyflie_tools/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pascal/dd2419_ws/devel/.private/crazyflie_tools/lib/crazyflie_tools/reboot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/reboot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/reboot.dir/build: /home/pascal/dd2419_ws/devel/.private/crazyflie_tools/lib/crazyflie_tools/reboot

.PHONY : CMakeFiles/reboot.dir/build

CMakeFiles/reboot.dir/requires: CMakeFiles/reboot.dir/src/reboot.cpp.o.requires

.PHONY : CMakeFiles/reboot.dir/requires

CMakeFiles/reboot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/reboot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/reboot.dir/clean

CMakeFiles/reboot.dir/depend:
	cd /home/pascal/dd2419_ws/build/crazyflie_tools && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_tools /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_tools /home/pascal/dd2419_ws/build/crazyflie_tools /home/pascal/dd2419_ws/build/crazyflie_tools /home/pascal/dd2419_ws/build/crazyflie_tools/CMakeFiles/reboot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/reboot.dir/depend

