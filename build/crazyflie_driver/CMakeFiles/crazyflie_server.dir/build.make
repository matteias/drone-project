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
CMAKE_SOURCE_DIR = /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pascal/dd2419_ws/build/crazyflie_driver

# Include any dependencies generated for this target.
include CMakeFiles/crazyflie_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/crazyflie_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/crazyflie_server.dir/flags.make

CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o: CMakeFiles/crazyflie_server.dir/flags.make
CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o: /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pascal/dd2419_ws/build/crazyflie_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o -c /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp

CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp > CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.i

CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp -o CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.s

CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires:

.PHONY : CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires

CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides: CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires
	$(MAKE) -f CMakeFiles/crazyflie_server.dir/build.make CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides.build
.PHONY : CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides

CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides.build: CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o


# Object files for target crazyflie_server
crazyflie_server_OBJECTS = \
"CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o"

# External object files for target crazyflie_server
crazyflie_server_EXTERNAL_OBJECTS =

/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: CMakeFiles/crazyflie_server.dir/build.make
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libtf.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libtf2_ros.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libactionlib.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libmessage_filters.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libroscpp.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libtf2.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/librosconsole.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/librostime.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /opt/ros/melodic/lib/libcpp_common.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: /home/pascal/dd2419_ws/devel/.private/crazyflie_cpp/lib/libcrazyflie_cpp.so
/home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server: CMakeFiles/crazyflie_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pascal/dd2419_ws/build/crazyflie_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/crazyflie_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/crazyflie_server.dir/build: /home/pascal/dd2419_ws/devel/.private/crazyflie_driver/lib/crazyflie_driver/crazyflie_server

.PHONY : CMakeFiles/crazyflie_server.dir/build

CMakeFiles/crazyflie_server.dir/requires: CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires

.PHONY : CMakeFiles/crazyflie_server.dir/requires

CMakeFiles/crazyflie_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/crazyflie_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/crazyflie_server.dir/clean

CMakeFiles/crazyflie_server.dir/depend:
	cd /home/pascal/dd2419_ws/build/crazyflie_driver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver /home/pascal/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver /home/pascal/dd2419_ws/build/crazyflie_driver /home/pascal/dd2419_ws/build/crazyflie_driver /home/pascal/dd2419_ws/build/crazyflie_driver/CMakeFiles/crazyflie_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/crazyflie_server.dir/depend

