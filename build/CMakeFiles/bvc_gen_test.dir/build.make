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
CMAKE_SOURCE_DIR = /home/changju/cpp_ws/subgoal_generator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/changju/cpp_ws/subgoal_generator/build

# Include any dependencies generated for this target.
include CMakeFiles/bvc_gen_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bvc_gen_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bvc_gen_test.dir/flags.make

CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.o: CMakeFiles/bvc_gen_test.dir/flags.make
CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.o: ../test/bvc_generator_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/changju/cpp_ws/subgoal_generator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.o -c /home/changju/cpp_ws/subgoal_generator/test/bvc_generator_test.cpp

CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/changju/cpp_ws/subgoal_generator/test/bvc_generator_test.cpp > CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.i

CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/changju/cpp_ws/subgoal_generator/test/bvc_generator_test.cpp -o CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.s

# Object files for target bvc_gen_test
bvc_gen_test_OBJECTS = \
"CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.o"

# External object files for target bvc_gen_test
bvc_gen_test_EXTERNAL_OBJECTS =

bvc_gen_test: CMakeFiles/bvc_gen_test.dir/test/bvc_generator_test.cpp.o
bvc_gen_test: CMakeFiles/bvc_gen_test.dir/build.make
bvc_gen_test: lib/libgtest_main.a
bvc_gen_test: libsrc_lib.a
bvc_gen_test: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.6.2
bvc_gen_test: /usr/lib/x86_64-linux-gnu/libmpfr.so
bvc_gen_test: /usr/lib/x86_64-linux-gnu/libgmp.so
bvc_gen_test: lib/libgtest.a
bvc_gen_test: CMakeFiles/bvc_gen_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/changju/cpp_ws/subgoal_generator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bvc_gen_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bvc_gen_test.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/cmake -D TEST_TARGET=bvc_gen_test -D TEST_EXECUTABLE=/home/changju/cpp_ws/subgoal_generator/build/bvc_gen_test -D TEST_EXECUTOR= -D TEST_WORKING_DIR=/home/changju/cpp_ws/subgoal_generator/build -D TEST_EXTRA_ARGS= -D TEST_PROPERTIES= -D TEST_PREFIX= -D TEST_SUFFIX= -D NO_PRETTY_TYPES=FALSE -D NO_PRETTY_VALUES=FALSE -D TEST_LIST=bvc_gen_test_TESTS -D CTEST_FILE=/home/changju/cpp_ws/subgoal_generator/build/bvc_gen_test[1]_tests.cmake -D TEST_DISCOVERY_TIMEOUT=5 -P /usr/share/cmake-3.16/Modules/GoogleTestAddTests.cmake

# Rule to build all files generated by this target.
CMakeFiles/bvc_gen_test.dir/build: bvc_gen_test

.PHONY : CMakeFiles/bvc_gen_test.dir/build

CMakeFiles/bvc_gen_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bvc_gen_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bvc_gen_test.dir/clean

CMakeFiles/bvc_gen_test.dir/depend:
	cd /home/changju/cpp_ws/subgoal_generator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/changju/cpp_ws/subgoal_generator /home/changju/cpp_ws/subgoal_generator /home/changju/cpp_ws/subgoal_generator/build /home/changju/cpp_ws/subgoal_generator/build /home/changju/cpp_ws/subgoal_generator/build/CMakeFiles/bvc_gen_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bvc_gen_test.dir/depend

