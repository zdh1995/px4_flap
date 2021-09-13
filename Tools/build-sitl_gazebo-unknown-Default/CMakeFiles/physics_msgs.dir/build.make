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
CMAKE_SOURCE_DIR = /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default

# Include any dependencies generated for this target.
include CMakeFiles/physics_msgs.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/physics_msgs.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/physics_msgs.dir/flags.make

Wind.pb.cc: /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/msgs/Wind.proto
Wind.pb.cc: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running C++ protocol buffer compiler on msgs/Wind.proto"
	/usr/bin/protoc --cpp_out=/home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default -I /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/msgs -I /usr/include/gazebo-9/gazebo/msgs/proto /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/msgs/Wind.proto

Wind.pb.h: Wind.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate Wind.pb.h

CMakeFiles/physics_msgs.dir/Wind.pb.cc.o: CMakeFiles/physics_msgs.dir/flags.make
CMakeFiles/physics_msgs.dir/Wind.pb.cc.o: Wind.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/physics_msgs.dir/Wind.pb.cc.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/physics_msgs.dir/Wind.pb.cc.o -c /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/Wind.pb.cc

CMakeFiles/physics_msgs.dir/Wind.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/physics_msgs.dir/Wind.pb.cc.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/Wind.pb.cc > CMakeFiles/physics_msgs.dir/Wind.pb.cc.i

CMakeFiles/physics_msgs.dir/Wind.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/physics_msgs.dir/Wind.pb.cc.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/Wind.pb.cc -o CMakeFiles/physics_msgs.dir/Wind.pb.cc.s

CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.requires:

.PHONY : CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.requires

CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.provides: CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/physics_msgs.dir/build.make CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.provides.build
.PHONY : CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.provides

CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.provides.build: CMakeFiles/physics_msgs.dir/Wind.pb.cc.o


# Object files for target physics_msgs
physics_msgs_OBJECTS = \
"CMakeFiles/physics_msgs.dir/Wind.pb.cc.o"

# External object files for target physics_msgs
physics_msgs_EXTERNAL_OBJECTS =

libphysics_msgs.so: CMakeFiles/physics_msgs.dir/Wind.pb.cc.o
libphysics_msgs.so: CMakeFiles/physics_msgs.dir/build.make
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libblas.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libblas.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-transport4.so.4.0.0
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-common1.so.1.1.1
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_dnn.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_gapi.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_highgui.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_ml.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_objdetect.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_photo.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_stitching.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_video.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_videoio.so.4.2.0
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libblas.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libphysics_msgs.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libphysics_msgs.so: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_calib3d.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_features2d.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_flann.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_imgproc.so.4.2.0
libphysics_msgs.so: /usr/local/lib/libopencv_core.so.4.2.0
libphysics_msgs.so: CMakeFiles/physics_msgs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libphysics_msgs.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/physics_msgs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/physics_msgs.dir/build: libphysics_msgs.so

.PHONY : CMakeFiles/physics_msgs.dir/build

CMakeFiles/physics_msgs.dir/requires: CMakeFiles/physics_msgs.dir/Wind.pb.cc.o.requires

.PHONY : CMakeFiles/physics_msgs.dir/requires

CMakeFiles/physics_msgs.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/physics_msgs.dir/cmake_clean.cmake
.PHONY : CMakeFiles/physics_msgs.dir/clean

CMakeFiles/physics_msgs.dir/depend: Wind.pb.cc
CMakeFiles/physics_msgs.dir/depend: Wind.pb.h
	cd /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/CMakeFiles/physics_msgs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/physics_msgs.dir/depend
