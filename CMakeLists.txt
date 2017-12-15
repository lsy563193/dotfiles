cmake_minimum_required(VERSION 2.8.3)
project(pp)
set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")
FILE(GLOB PP_SRC
        include/*
				include/path_algorithm.h
		include/action.hpp
#	src/charge.cpp
#	src/clean_mode.cpp
#	src/clean_state.cpp
#	src/core_move.cpp
	src/des.cpp

	src/dev/gyro.cpp
	src/dev/vacuum.cpp
	src/dev/key.cpp
	src/dev/cliff.cpp
	src/dev/battery.cpp
	src/dev/brush.cpp
	src/dev/bumper.cpp
	src/dev/rcon.cpp
		src/dev/robot_timer.cpp
	src/dev/remote.cpp
	src/dev/obs.cpp
	src/dev/led.cpp
	src/dev/beep.cpp
	src/dev/charger.cpp
	src/dev/wall_follow.cpp
	src/dev/wheel.cpp
	src/dev/speaker.cpp
	src/dev/lidar.cpp

#				src/action/action_new.cpp
#				src/action/action_open_gyro_new.cpp
				src/mode/state/state.cpp
				src/mode/state/state_clean.cpp
				src/mode/state/state_go_home_point.cpp
				src/mode/state/state_go_charger.cpp
				src/mode/state/state_trapped.cpp
				src/mode/state/state_self_check.cpp
				src/mode/state/state_tmp_spot.cpp
				src/mode/state/state_exploration.cpp

				src/action/move_type/move_type.cpp
				src/action/move_type/action_linear.cpp
				src/action/move_type/action_follow_wall.cpp
				src/action/move_type/action_go_charger.cpp
	src/mode/mode.cpp
	src/mode/mode_sleep.cpp
	src/mode/mode_charge.cpp
	src/mode/mode_idle.cpp
	src/mode/mode_remote.cpp
	src/mode/clean_mode.cpp
	src/mode/clean_mode_navigation.cpp
	src/mode/clean_mode_follow_wall.cpp
				src/mode/clean_mode_spot.cpp
	src/path_algorithm/path_algorithm.cpp
				src/path_algorithm/path_algorithm_navigation_clean.cpp
				src/path_algorithm/path_algorithm_follow_wall.cpp
				src/path_algorithm/path_algorithm_spot.cpp
				src/path_algorithm/path_algorithm_go_home.cpp
				src/path_algorithm/shortest_path_algorithm.cpp

	src/speed_governor/*
				src/global.cpp

	src/action/action.cpp
	src/action/action_open_gyro.cpp
	src/action/action_back_from_charger.cpp
	src/action/action_open_lidar.cpp
	src/action/action_align.cpp
	src/action/action_open_slam.cpp
	src/action/action_sleep.cpp
	src/action/action_idle.cpp
	src/action/action_charge.cpp
	src/action/action_pause.cpp

				src/action/move_type/movement/movement.cpp
				src/action/move_type/movement/movement.cpp
				src/action/move_type/movement/movement_back.cpp
				src/action/move_type/movement/movement_back.cpp
				src/action/move_type/movement/movement_fall_wall.cpp
				src/action/move_type/movement/movement_fall_wall.cpp
				src/action/move_type/movement/movement_forward.cpp
				src/action/move_type/movement/movement_forward.cpp
				src/action/move_type/movement/movement_go_charger.cpp
				src/action/move_type/movement/movement_go_charger.cpp
#				src/action/move_type/movement/movement_self_check.cpp
        src/action/move_type/movement/movement_exception_resume.cpp
				src/action/move_type/movement/movement_turn.cpp
				src/action/move_type/movement/movement_turn.cpp
				src/action/move_type/movement/movement_turn_for_charger.cpp
				src/action/move_type/movement/movement_stay.cpp
				src/action/move_type/movement/movement_direct_go.cpp

#	src/clean_mode/clean_mode.cpp
#	src/clean_mode/exploration.cpp
#	src/clean_mode/navigation_clean.cpp
#	src/clean_mode/spot_clean.cpp
#	src/clean_mode/wall_follow_clean.cpp

	src/error.cpp
	src/event_manager.cpp
#        src/idle.cpp
	src/main.cpp
	src/map.cpp
	src/mathematics.cpp
#	src/motion_manage.cpp
#	src/move_type.cpp
#	src/movement.cpp
	src/odom.cpp
#	src/path_planning.cpp
	src/pose.cpp
#	src/regulator.cpp
#	src/remote_mode.cpp
	src/robot.cpp
	src/robotbase.cpp
	src/serial.cpp
#	src/shortest_path.cpp
	src/signature.c
	src/slam.cpp
	src/slam_map.cpp
#	src/sleep.cpp
#	src/space_exploration.cpp
#	src/spot.cpp
	src/verify.cpp
)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	rosconsole
	roscpp
	rospy
	sensor_msgs
	std_srvs
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
	scan_ctrl.msg
)

add_service_files(
	DIRECTORY srv
	FILES
	SetLidar.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	std_srvs
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
	std_srvs
	visualization_msgs
	geometry_msgs
	sensor_msgs
	tf
	DEPENDS Eigen3
)

add_executable(pp  ${PP_SRC})
#target_link_libraries(pp ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${Eigen3_LIBRARIES} -lm -lpthread)
target_link_libraries(pp ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${Eigen3_LIBRARIES} -lm -lpthread "-T ../ilife_ldscript.lds ../res.o -Wl,--verbose")

#add_executable(syscheck src/signature.c src/crc8.cpp src/des.cpp src/serial.cpp src/syscheck.cpp)
#target_link_libraries(syscheck ${catkin_LIBRARIES} -lm "-T ../ilife_ldscript.lds -Wl,--verbose")
#
#add_executable(reg_test ${PP_SRC} )
#target_link_libraries(reg_test ${catkin_LIBRARIES} -lm -lpthread)
#target_link_libraries(reg_test ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${Eigen3_LIBRARIES} -lm -lpthread)

#install(TARGETS pp syscheck
install(TARGETS pp
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
