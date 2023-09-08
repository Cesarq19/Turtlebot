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
CMAKE_SOURCE_DIR = /home/cesar/turtlebot3_ws/src/YDLidar-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10

# Include any dependencies generated for this target.
include CMakeFiles/ydlidar_sdk.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ydlidar_sdk.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ydlidar_sdk.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ydlidar_sdk.dir/flags.make

CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o: ../../core/base/timer.cpp
CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/base/timer.cpp

CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/base/timer.cpp > CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/base/timer.cpp -o CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.s

CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o: ../../core/common/ydlidar_def.cpp
CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/common/ydlidar_def.cpp

CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/common/ydlidar_def.cpp > CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/common/ydlidar_def.cpp -o CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.s

CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o: ../../core/network/ActiveSocket.cpp
CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/ActiveSocket.cpp

CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/ActiveSocket.cpp > CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/ActiveSocket.cpp -o CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.s

CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o: ../../core/network/PassiveSocket.cpp
CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/PassiveSocket.cpp

CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/PassiveSocket.cpp > CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/PassiveSocket.cpp -o CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.s

CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o: ../../core/network/SimpleSocket.cpp
CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/SimpleSocket.cpp

CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/SimpleSocket.cpp > CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/network/SimpleSocket.cpp -o CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.s

CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o: ../../core/serial/serial.cpp
CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/serial.cpp

CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/serial.cpp > CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/serial.cpp -o CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.s

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o: ../../core/serial/impl/unix/lock.c
CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o -MF CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o.d -o CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/lock.c

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/lock.c > CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.i

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/lock.c -o CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.s

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o: ../../core/serial/impl/unix/list_ports_linux.cpp
CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/list_ports_linux.cpp

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/list_ports_linux.cpp > CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/list_ports_linux.cpp -o CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.s

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o: ../../core/serial/impl/unix/unix_serial.cpp
CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/unix_serial.cpp

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/unix_serial.cpp > CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.i

CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/core/serial/impl/unix/unix_serial.cpp -o CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.s

CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o: ../../src/CYdLidar.cpp
CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/CYdLidar.cpp

CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/CYdLidar.cpp > CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.i

CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/CYdLidar.cpp -o CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.s

CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o: ../../src/ETLidarDriver.cpp
CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ETLidarDriver.cpp

CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ETLidarDriver.cpp > CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.i

CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ETLidarDriver.cpp -o CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.s

CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o: ../../src/ydlidar_driver.cpp
CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ydlidar_driver.cpp

CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ydlidar_driver.cpp > CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.i

CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ydlidar_driver.cpp -o CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.s

CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o: CMakeFiles/ydlidar_sdk.dir/flags.make
CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o: ../../src/ydlidar_sdk.cpp
CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o: CMakeFiles/ydlidar_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o -MF CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o.d -o CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o -c /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ydlidar_sdk.cpp

CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ydlidar_sdk.cpp > CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.i

CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cesar/turtlebot3_ws/src/YDLidar-SDK/src/ydlidar_sdk.cpp -o CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.s

# Object files for target ydlidar_sdk
ydlidar_sdk_OBJECTS = \
"CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o" \
"CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o" \
"CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o"

# External object files for target ydlidar_sdk
ydlidar_sdk_EXTERNAL_OBJECTS =

libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/base/timer.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/common/ydlidar_def.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/network/ActiveSocket.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/network/PassiveSocket.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/network/SimpleSocket.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/serial/serial.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/lock.c.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/list_ports_linux.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/core/serial/impl/unix/unix_serial.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/src/CYdLidar.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/src/ETLidarDriver.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/src/ydlidar_driver.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/src/ydlidar_sdk.cpp.o
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/build.make
libydlidar_sdk.a: CMakeFiles/ydlidar_sdk.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking CXX static library libydlidar_sdk.a"
	$(CMAKE_COMMAND) -P CMakeFiles/ydlidar_sdk.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ydlidar_sdk.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ydlidar_sdk.dir/build: libydlidar_sdk.a
.PHONY : CMakeFiles/ydlidar_sdk.dir/build

CMakeFiles/ydlidar_sdk.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ydlidar_sdk.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ydlidar_sdk.dir/clean

CMakeFiles/ydlidar_sdk.dir/depend:
	cd /home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cesar/turtlebot3_ws/src/YDLidar-SDK /home/cesar/turtlebot3_ws/src/YDLidar-SDK /home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10 /home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10 /home/cesar/turtlebot3_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles/ydlidar_sdk.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ydlidar_sdk.dir/depend

