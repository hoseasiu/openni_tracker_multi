# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hosea/ros_workspace/openni_tracker_ws

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hosea/ros_workspace/openni_tracker_ws/build

# Include any dependencies generated for this target.
include CMakeFiles/openni_tracker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/openni_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openni_tracker.dir/flags.make

CMakeFiles/openni_tracker.dir/src/openni_tracker.o: CMakeFiles/openni_tracker.dir/flags.make
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: ../src/openni_tracker.cpp
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: ../manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/orocos_kdl/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/python_orocos_kdl/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/kdl/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/openni_tracker.dir/src/openni_tracker.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hosea/ros_workspace/openni_tracker_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/openni_tracker.dir/src/openni_tracker.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/openni_tracker.dir/src/openni_tracker.o -c /home/hosea/ros_workspace/openni_tracker_ws/src/openni_tracker.cpp

CMakeFiles/openni_tracker.dir/src/openni_tracker.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openni_tracker.dir/src/openni_tracker.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/hosea/ros_workspace/openni_tracker_ws/src/openni_tracker.cpp > CMakeFiles/openni_tracker.dir/src/openni_tracker.i

CMakeFiles/openni_tracker.dir/src/openni_tracker.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openni_tracker.dir/src/openni_tracker.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/hosea/ros_workspace/openni_tracker_ws/src/openni_tracker.cpp -o CMakeFiles/openni_tracker.dir/src/openni_tracker.s

CMakeFiles/openni_tracker.dir/src/openni_tracker.o.requires:
.PHONY : CMakeFiles/openni_tracker.dir/src/openni_tracker.o.requires

CMakeFiles/openni_tracker.dir/src/openni_tracker.o.provides: CMakeFiles/openni_tracker.dir/src/openni_tracker.o.requires
	$(MAKE) -f CMakeFiles/openni_tracker.dir/build.make CMakeFiles/openni_tracker.dir/src/openni_tracker.o.provides.build
.PHONY : CMakeFiles/openni_tracker.dir/src/openni_tracker.o.provides

CMakeFiles/openni_tracker.dir/src/openni_tracker.o.provides.build: CMakeFiles/openni_tracker.dir/src/openni_tracker.o

# Object files for target openni_tracker
openni_tracker_OBJECTS = \
"CMakeFiles/openni_tracker.dir/src/openni_tracker.o"

# External object files for target openni_tracker
openni_tracker_EXTERNAL_OBJECTS =

../bin/openni_tracker: CMakeFiles/openni_tracker.dir/src/openni_tracker.o
../bin/openni_tracker: CMakeFiles/openni_tracker.dir/build.make
../bin/openni_tracker: CMakeFiles/openni_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/openni_tracker"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openni_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openni_tracker.dir/build: ../bin/openni_tracker
.PHONY : CMakeFiles/openni_tracker.dir/build

CMakeFiles/openni_tracker.dir/requires: CMakeFiles/openni_tracker.dir/src/openni_tracker.o.requires
.PHONY : CMakeFiles/openni_tracker.dir/requires

CMakeFiles/openni_tracker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openni_tracker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openni_tracker.dir/clean

CMakeFiles/openni_tracker.dir/depend:
	cd /home/hosea/ros_workspace/openni_tracker_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hosea/ros_workspace/openni_tracker_ws /home/hosea/ros_workspace/openni_tracker_ws /home/hosea/ros_workspace/openni_tracker_ws/build /home/hosea/ros_workspace/openni_tracker_ws/build /home/hosea/ros_workspace/openni_tracker_ws/build/CMakeFiles/openni_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openni_tracker.dir/depend

