# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_SOURCE_DIR = /home/pewilond/vs/lab4_4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pewilond/vs/lab4_4/build

# Include any dependencies generated for this target.
include CMakeFiles/lab4_4.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/lab4_4.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/lab4_4.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lab4_4.dir/flags.make

lab4_4_autogen/timestamp: /usr/lib/qt5/bin/moc
lab4_4_autogen/timestamp: /usr/lib/qt5/bin/uic
lab4_4_autogen/timestamp: CMakeFiles/lab4_4.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/pewilond/vs/lab4_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target lab4_4"
	/usr/bin/cmake -E cmake_autogen /home/pewilond/vs/lab4_4/build/CMakeFiles/lab4_4_autogen.dir/AutogenInfo.json ""
	/usr/bin/cmake -E touch /home/pewilond/vs/lab4_4/build/lab4_4_autogen/timestamp

CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o: CMakeFiles/lab4_4.dir/flags.make
CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o: lab4_4_autogen/mocs_compilation.cpp
CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o: CMakeFiles/lab4_4.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pewilond/vs/lab4_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o -MF CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o -c /home/pewilond/vs/lab4_4/build/lab4_4_autogen/mocs_compilation.cpp

CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pewilond/vs/lab4_4/build/lab4_4_autogen/mocs_compilation.cpp > CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.i

CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pewilond/vs/lab4_4/build/lab4_4_autogen/mocs_compilation.cpp -o CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.s

CMakeFiles/lab4_4.dir/main.cpp.o: CMakeFiles/lab4_4.dir/flags.make
CMakeFiles/lab4_4.dir/main.cpp.o: /home/pewilond/vs/lab4_4/main.cpp
CMakeFiles/lab4_4.dir/main.cpp.o: CMakeFiles/lab4_4.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pewilond/vs/lab4_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/lab4_4.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lab4_4.dir/main.cpp.o -MF CMakeFiles/lab4_4.dir/main.cpp.o.d -o CMakeFiles/lab4_4.dir/main.cpp.o -c /home/pewilond/vs/lab4_4/main.cpp

CMakeFiles/lab4_4.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/lab4_4.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pewilond/vs/lab4_4/main.cpp > CMakeFiles/lab4_4.dir/main.cpp.i

CMakeFiles/lab4_4.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/lab4_4.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pewilond/vs/lab4_4/main.cpp -o CMakeFiles/lab4_4.dir/main.cpp.s

CMakeFiles/lab4_4.dir/display_window.cpp.o: CMakeFiles/lab4_4.dir/flags.make
CMakeFiles/lab4_4.dir/display_window.cpp.o: /home/pewilond/vs/lab4_4/display_window.cpp
CMakeFiles/lab4_4.dir/display_window.cpp.o: CMakeFiles/lab4_4.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pewilond/vs/lab4_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/lab4_4.dir/display_window.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lab4_4.dir/display_window.cpp.o -MF CMakeFiles/lab4_4.dir/display_window.cpp.o.d -o CMakeFiles/lab4_4.dir/display_window.cpp.o -c /home/pewilond/vs/lab4_4/display_window.cpp

CMakeFiles/lab4_4.dir/display_window.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/lab4_4.dir/display_window.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pewilond/vs/lab4_4/display_window.cpp > CMakeFiles/lab4_4.dir/display_window.cpp.i

CMakeFiles/lab4_4.dir/display_window.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/lab4_4.dir/display_window.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pewilond/vs/lab4_4/display_window.cpp -o CMakeFiles/lab4_4.dir/display_window.cpp.s

CMakeFiles/lab4_4.dir/test.cpp.o: CMakeFiles/lab4_4.dir/flags.make
CMakeFiles/lab4_4.dir/test.cpp.o: /home/pewilond/vs/lab4_4/test.cpp
CMakeFiles/lab4_4.dir/test.cpp.o: CMakeFiles/lab4_4.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pewilond/vs/lab4_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/lab4_4.dir/test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lab4_4.dir/test.cpp.o -MF CMakeFiles/lab4_4.dir/test.cpp.o.d -o CMakeFiles/lab4_4.dir/test.cpp.o -c /home/pewilond/vs/lab4_4/test.cpp

CMakeFiles/lab4_4.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/lab4_4.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pewilond/vs/lab4_4/test.cpp > CMakeFiles/lab4_4.dir/test.cpp.i

CMakeFiles/lab4_4.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/lab4_4.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pewilond/vs/lab4_4/test.cpp -o CMakeFiles/lab4_4.dir/test.cpp.s

# Object files for target lab4_4
lab4_4_OBJECTS = \
"CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/lab4_4.dir/main.cpp.o" \
"CMakeFiles/lab4_4.dir/display_window.cpp.o" \
"CMakeFiles/lab4_4.dir/test.cpp.o"

# External object files for target lab4_4
lab4_4_EXTERNAL_OBJECTS =

lab4_4: CMakeFiles/lab4_4.dir/lab4_4_autogen/mocs_compilation.cpp.o
lab4_4: CMakeFiles/lab4_4.dir/main.cpp.o
lab4_4: CMakeFiles/lab4_4.dir/display_window.cpp.o
lab4_4: CMakeFiles/lab4_4.dir/test.cpp.o
lab4_4: CMakeFiles/lab4_4.dir/build.make
lab4_4: /usr/lib/x86_64-linux-gnu/libQt5Charts.so.5.15.13
lab4_4: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.13
lab4_4: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.13
lab4_4: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.13
lab4_4: CMakeFiles/lab4_4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/pewilond/vs/lab4_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable lab4_4"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lab4_4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lab4_4.dir/build: lab4_4
.PHONY : CMakeFiles/lab4_4.dir/build

CMakeFiles/lab4_4.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lab4_4.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lab4_4.dir/clean

CMakeFiles/lab4_4.dir/depend: lab4_4_autogen/timestamp
	cd /home/pewilond/vs/lab4_4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pewilond/vs/lab4_4 /home/pewilond/vs/lab4_4 /home/pewilond/vs/lab4_4/build /home/pewilond/vs/lab4_4/build /home/pewilond/vs/lab4_4/build/CMakeFiles/lab4_4.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/lab4_4.dir/depend

