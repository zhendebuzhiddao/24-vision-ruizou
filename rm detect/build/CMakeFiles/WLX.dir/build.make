# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/buzhidao/Desktop/rm detect"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/buzhidao/Desktop/rm detect/build"

# Include any dependencies generated for this target.
include CMakeFiles/WLX.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/WLX.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/WLX.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/WLX.dir/flags.make

CMakeFiles/WLX.dir/main.cpp.o: CMakeFiles/WLX.dir/flags.make
CMakeFiles/WLX.dir/main.cpp.o: ../main.cpp
CMakeFiles/WLX.dir/main.cpp.o: CMakeFiles/WLX.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/buzhidao/Desktop/rm detect/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/WLX.dir/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WLX.dir/main.cpp.o -MF CMakeFiles/WLX.dir/main.cpp.o.d -o CMakeFiles/WLX.dir/main.cpp.o -c "/home/buzhidao/Desktop/rm detect/main.cpp"

CMakeFiles/WLX.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/WLX.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/buzhidao/Desktop/rm detect/main.cpp" > CMakeFiles/WLX.dir/main.cpp.i

CMakeFiles/WLX.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/WLX.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/buzhidao/Desktop/rm detect/main.cpp" -o CMakeFiles/WLX.dir/main.cpp.s

# Object files for target WLX
WLX_OBJECTS = \
"CMakeFiles/WLX.dir/main.cpp.o"

# External object files for target WLX
WLX_EXTERNAL_OBJECTS =

WLX: CMakeFiles/WLX.dir/main.cpp.o
WLX: CMakeFiles/WLX.dir/build.make
WLX: /usr/local/lib/libopencv_gapi.so.4.7.0
WLX: /usr/local/lib/libopencv_highgui.so.4.7.0
WLX: /usr/local/lib/libopencv_ml.so.4.7.0
WLX: /usr/local/lib/libopencv_objdetect.so.4.7.0
WLX: /usr/local/lib/libopencv_photo.so.4.7.0
WLX: /usr/local/lib/libopencv_stitching.so.4.7.0
WLX: /usr/local/lib/libopencv_video.so.4.7.0
WLX: /usr/local/lib/libopencv_videoio.so.4.7.0
WLX: /usr/local/lib/libopencv_imgcodecs.so.4.7.0
WLX: /usr/local/lib/libopencv_dnn.so.4.7.0
WLX: /usr/local/lib/libopencv_calib3d.so.4.7.0
WLX: /usr/local/lib/libopencv_features2d.so.4.7.0
WLX: /usr/local/lib/libopencv_flann.so.4.7.0
WLX: /usr/local/lib/libopencv_imgproc.so.4.7.0
WLX: /usr/local/lib/libopencv_core.so.4.7.0
WLX: CMakeFiles/WLX.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/buzhidao/Desktop/rm detect/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable WLX"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/WLX.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/WLX.dir/build: WLX
.PHONY : CMakeFiles/WLX.dir/build

CMakeFiles/WLX.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/WLX.dir/cmake_clean.cmake
.PHONY : CMakeFiles/WLX.dir/clean

CMakeFiles/WLX.dir/depend:
	cd "/home/buzhidao/Desktop/rm detect/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/buzhidao/Desktop/rm detect" "/home/buzhidao/Desktop/rm detect" "/home/buzhidao/Desktop/rm detect/build" "/home/buzhidao/Desktop/rm detect/build" "/home/buzhidao/Desktop/rm detect/build/CMakeFiles/WLX.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/WLX.dir/depend

