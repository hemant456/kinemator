# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/hemant/surabhilabs/kinemator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hemant/surabhilabs/kinemator

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/hemant/surabhilabs/kinemator/CMakeFiles /home/hemant/surabhilabs/kinemator/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/hemant/surabhilabs/kinemator/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named kinemator

# Build rule for target.
kinemator: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 kinemator
.PHONY : kinemator

# fast build rule for target.
kinemator/fast:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/build
.PHONY : kinemator/fast

src/ImagePlanner.o: src/ImagePlanner.cpp.o

.PHONY : src/ImagePlanner.o

# target to build an object file
src/ImagePlanner.cpp.o:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/ImagePlanner.cpp.o
.PHONY : src/ImagePlanner.cpp.o

src/ImagePlanner.i: src/ImagePlanner.cpp.i

.PHONY : src/ImagePlanner.i

# target to preprocess a source file
src/ImagePlanner.cpp.i:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/ImagePlanner.cpp.i
.PHONY : src/ImagePlanner.cpp.i

src/ImagePlanner.s: src/ImagePlanner.cpp.s

.PHONY : src/ImagePlanner.s

# target to generate assembly for a file
src/ImagePlanner.cpp.s:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/ImagePlanner.cpp.s
.PHONY : src/ImagePlanner.cpp.s

src/Kinemator.o: src/Kinemator.cpp.o

.PHONY : src/Kinemator.o

# target to build an object file
src/Kinemator.cpp.o:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/Kinemator.cpp.o
.PHONY : src/Kinemator.cpp.o

src/Kinemator.i: src/Kinemator.cpp.i

.PHONY : src/Kinemator.i

# target to preprocess a source file
src/Kinemator.cpp.i:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/Kinemator.cpp.i
.PHONY : src/Kinemator.cpp.i

src/Kinemator.s: src/Kinemator.cpp.s

.PHONY : src/Kinemator.s

# target to generate assembly for a file
src/Kinemator.cpp.s:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/Kinemator.cpp.s
.PHONY : src/Kinemator.cpp.s

src/main.o: src/main.cpp.o

.PHONY : src/main.o

# target to build an object file
src/main.cpp.o:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/main.cpp.o
.PHONY : src/main.cpp.o

src/main.i: src/main.cpp.i

.PHONY : src/main.i

# target to preprocess a source file
src/main.cpp.i:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/main.cpp.i
.PHONY : src/main.cpp.i

src/main.s: src/main.cpp.s

.PHONY : src/main.s

# target to generate assembly for a file
src/main.cpp.s:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/main.cpp.s
.PHONY : src/main.cpp.s

src/math.o: src/math.cpp.o

.PHONY : src/math.o

# target to build an object file
src/math.cpp.o:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/math.cpp.o
.PHONY : src/math.cpp.o

src/math.i: src/math.cpp.i

.PHONY : src/math.i

# target to preprocess a source file
src/math.cpp.i:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/math.cpp.i
.PHONY : src/math.cpp.i

src/math.s: src/math.cpp.s

.PHONY : src/math.s

# target to generate assembly for a file
src/math.cpp.s:
	$(MAKE) -f CMakeFiles/kinemator.dir/build.make CMakeFiles/kinemator.dir/src/math.cpp.s
.PHONY : src/math.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... kinemator"
	@echo "... edit_cache"
	@echo "... src/ImagePlanner.o"
	@echo "... src/ImagePlanner.i"
	@echo "... src/ImagePlanner.s"
	@echo "... src/Kinemator.o"
	@echo "... src/Kinemator.i"
	@echo "... src/Kinemator.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
	@echo "... src/math.o"
	@echo "... src/math.i"
	@echo "... src/math.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

