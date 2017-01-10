cmake_minimum_required(VERSION 2.8)
project( px4_berry_eyes )
# you can see the cmake output on the command line by doing
# set( CMAKE_VERBOSE_MAKEFILE on )

include_directories( ${px4_berry_eyes_SOURCE_DIR}/mavlink/include/mavlink/v1.0 )

set( raspicam_DIR "/usr/local/lib/cmake" )

set( SOURCES 
	src/autopilot_interface.cpp
	src/mavlink_control.cpp
	src/serial_port.cpp
	src/camera_interface.cpp )

find_package( raspicam REQUIRED )

find_package( OpenCV REQUIRED )

find_package( Threads )

add_executable( px4_berry_eyes ${SOURCES} )

target_link_libraries( px4_berry_eyes ${CMAKE_THREAD_LIBS_INIT} )
target_link_libraries( px4_berry_eyes ${raspicam_CV_LIBS} )