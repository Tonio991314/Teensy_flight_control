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
CMAKE_SOURCE_DIR = /home/drone/Arduino/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/drone/Arduino/build

# Utility rule file for rosserial_test_rosserial_lib.

# Include the progress variables for this target.
include rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/progress.make

rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib: rosserial/rosserial_test/include/rosserial


rosserial/rosserial_test/include/rosserial:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/drone/Arduino/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating include/rosserial"
	cd /home/drone/Arduino/build/rosserial/rosserial_test && /home/drone/Arduino/devel/env.sh /usr/bin/python2 /home/drone/Arduino/src/rosserial/rosserial_test/scripts/generate_client_ros_lib.py /home/drone/Arduino/build/rosserial/rosserial_test/include

rosserial_test_rosserial_lib: rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib
rosserial_test_rosserial_lib: rosserial/rosserial_test/include/rosserial
rosserial_test_rosserial_lib: rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/build.make

.PHONY : rosserial_test_rosserial_lib

# Rule to build all files generated by this target.
rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/build: rosserial_test_rosserial_lib

.PHONY : rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/build

rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/clean:
	cd /home/drone/Arduino/build/rosserial/rosserial_test && $(CMAKE_COMMAND) -P CMakeFiles/rosserial_test_rosserial_lib.dir/cmake_clean.cmake
.PHONY : rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/clean

rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/depend:
	cd /home/drone/Arduino/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/drone/Arduino/src /home/drone/Arduino/src/rosserial/rosserial_test /home/drone/Arduino/build /home/drone/Arduino/build/rosserial/rosserial_test /home/drone/Arduino/build/rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rosserial/rosserial_test/CMakeFiles/rosserial_test_rosserial_lib.dir/depend

