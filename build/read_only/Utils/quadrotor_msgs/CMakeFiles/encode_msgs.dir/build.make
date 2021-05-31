# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/amov/motion_planning/Project/P_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amov/motion_planning/Project/P_ws/build

# Include any dependencies generated for this target.
include read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/depend.make

# Include the progress variables for this target.
include read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/progress.make

# Include the compile flags for this target's objects.
include read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/flags.make

read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o: read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/flags.make
read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o: /home/amov/motion_planning/Project/P_ws/src/read_only/Utils/quadrotor_msgs/src/encode_msgs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amov/motion_planning/Project/P_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o"
	cd /home/amov/motion_planning/Project/P_ws/build/read_only/Utils/quadrotor_msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o -c /home/amov/motion_planning/Project/P_ws/src/read_only/Utils/quadrotor_msgs/src/encode_msgs.cpp

read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.i"
	cd /home/amov/motion_planning/Project/P_ws/build/read_only/Utils/quadrotor_msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amov/motion_planning/Project/P_ws/src/read_only/Utils/quadrotor_msgs/src/encode_msgs.cpp > CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.i

read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.s"
	cd /home/amov/motion_planning/Project/P_ws/build/read_only/Utils/quadrotor_msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amov/motion_planning/Project/P_ws/src/read_only/Utils/quadrotor_msgs/src/encode_msgs.cpp -o CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.s

# Object files for target encode_msgs
encode_msgs_OBJECTS = \
"CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o"

# External object files for target encode_msgs
encode_msgs_EXTERNAL_OBJECTS =

/home/amov/motion_planning/Project/P_ws/devel/lib/libencode_msgs.so: read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o
/home/amov/motion_planning/Project/P_ws/devel/lib/libencode_msgs.so: read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/build.make
/home/amov/motion_planning/Project/P_ws/devel/lib/libencode_msgs.so: read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amov/motion_planning/Project/P_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/amov/motion_planning/Project/P_ws/devel/lib/libencode_msgs.so"
	cd /home/amov/motion_planning/Project/P_ws/build/read_only/Utils/quadrotor_msgs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/encode_msgs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/build: /home/amov/motion_planning/Project/P_ws/devel/lib/libencode_msgs.so

.PHONY : read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/build

read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/clean:
	cd /home/amov/motion_planning/Project/P_ws/build/read_only/Utils/quadrotor_msgs && $(CMAKE_COMMAND) -P CMakeFiles/encode_msgs.dir/cmake_clean.cmake
.PHONY : read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/clean

read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/depend:
	cd /home/amov/motion_planning/Project/P_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amov/motion_planning/Project/P_ws/src /home/amov/motion_planning/Project/P_ws/src/read_only/Utils/quadrotor_msgs /home/amov/motion_planning/Project/P_ws/build /home/amov/motion_planning/Project/P_ws/build/read_only/Utils/quadrotor_msgs /home/amov/motion_planning/Project/P_ws/build/read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : read_only/Utils/quadrotor_msgs/CMakeFiles/encode_msgs.dir/depend
