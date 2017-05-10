cmake_minimum_required(VERSION 2.8.3)
project(pp)
set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")
FILE(GLOB PP_SRC
	src/signature.c
	src/charger.cpp
	src/core_move.cpp
	src/crc8.cpp
	src/curve_move.cpp
	src/debug.cpp
	src/des.cpp
	src/gyro.cpp
	src/laser.cpp
	src/main.cpp
	src/map.cpp
	src/mathematics.cpp
	src/motion_controler.cpp
	src/movement.cpp
	src/path_planning.cpp
	src/random_runing.cpp
	src/remote_mode.cpp
	src/robotbase.cpp
	src/robot.cpp
	src/rounding.cpp
	src/serial.cpp
	src/shortest_path.cpp
	src/sleep.cpp
	src/spiral.cpp
	src/spot.cpp
	src/user_interface.cpp
	src/verify.cpp
	src/wall_follow_multi.cpp
	src/wall_follow_short.cpp
	src/wav.cpp
	src/slam.cpp
)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	rosconsole
	roscpp
	rospy
	sensor_msgs
	std_msgs
	geometry_msgs
	visualization_msgs
    cmake_modules
	tf
)

find_package(PkgConfig)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
pkg_check_modules(ALSA alsa REQUIRED)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigin3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Eigen3_LIBRARY_DIRS})

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
  ${ALSA_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_message_files(
	DIRECTORY msg
	FILES
	x900sensor.msg
	slam_angle_offset.msg
	robotmode.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	visualization_msgs
	geometry_msgs
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
	roscpp
	rospy
	std_msgs
	visualization_msgs
	geometry_msgs
	sensor_msgs
	tf
	DEPENDS Eigen3
)

add_executable(pp  ${PP_SRC} )
target_link_libraries(pp ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${Eigen3_LIBRARIES} -lm -lpthread "-T ../ilife_ldscript.lds ../res.o -Wl,--verbose")

add_executable(syscheck src/signature.c src/crc8.cpp src/des.cpp src/serial.cpp src/syscheck.cpp)
target_link_libraries(syscheck ${catkin_LIBRARIES} -lm "-T ../ilife_ldscript.lds -Wl,--verbose")

#add_executable(angle_test test/angle_test.cpp )
#target_link_libraries(angle_test ${catkin_LIBRARIES} -lm -lpthread)

install(TARGETS pp syscheck
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf config audio
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES custom_rosconsole.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
