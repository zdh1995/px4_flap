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

# Utility rule file for sdf.

# Include the progress variables for this target.
include CMakeFiles/sdf.dir/progress.make

CMakeFiles/sdf: /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf


/home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf: /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris.xacro
/home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf: /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.xacro
/home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf: /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/component_snippets.xacro
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf"
	cd /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo && rm -f /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf
	cd /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo && /usr/bin/python /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/scripts/xacro.py -o /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.urdf /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.xacro enable_mavlink_interface:=true enable_ground_truth:=false enable_wind:=false enable_logging:=false rotors_description_dir:=/home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description send_vision_estimation:=false send_odometry:=false
	cd /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo && gz sdf -p /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.urdf >> /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf
	cd /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo && rm -f /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.urdf

sdf: CMakeFiles/sdf
sdf: /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf
sdf: CMakeFiles/sdf.dir/build.make

.PHONY : sdf

# Rule to build all files generated by this target.
CMakeFiles/sdf.dir/build: sdf

.PHONY : CMakeFiles/sdf.dir/build

CMakeFiles/sdf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sdf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sdf.dir/clean

CMakeFiles/sdf.dir/depend:
	cd /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo /home/zdh/px4v1.10/Firmware/Tools/sitl_gazebo /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default /home/zdh/px4v1.10/Firmware/Tools/build-sitl_gazebo-unknown-Default/CMakeFiles/sdf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdf.dir/depend

