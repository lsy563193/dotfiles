cmake_minimum_required(VERSION 2.8.3)
project(pp)
set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors ")
FILE(GLOB PP_SRC
	include/*

	src/dev/gyro.cpp
	src/dev/vacuum.cpp
	src/dev/key.cpp
	src/dev/cliff.cpp
	src/dev/battery.cpp
	src/dev/brush.cpp
	src/dev/bumper.cpp
	src/dev/rcon.cpp
	src/dev/remote.cpp
	src/dev/obs.cpp
	src/dev/key_led.cpp
	src/dev/beeper.cpp
	src/dev/charger.cpp
	src/dev/wall_follow.cpp
	src/dev/wheel.cpp
	src/dev/speaker.cpp
	src/dev/lidar.cpp
	src/dev/water_tank.cpp
	src/dev/wifi/wifi.cpp
	src/dev/infrared_display.cpp
	src/dev/wifi_led.cpp

	src/appointment.cpp
	src/robot_timer.cpp

	src/mode/state/state.cpp
	src/mode/state/state_init.cpp
	src/mode/state/state_resume_low_battery_charge.cpp
	src/mode/state/state_pause.cpp
	src/mode/state/state_charge.cpp
	src/mode/state/state_clean.cpp
	src/mode/state/state_go_home_point.cpp
	src/mode/state/state_go_charger.cpp
	src/mode/state/state_follow_wall.cpp
	src/mode/state/state_exception_resume.cpp
	src/mode/state/state_spot.cpp
	src/mode/state/state_exploration.cpp
	src/mode/state/state_test.cpp

	src/mode/mode.cpp
	src/mode/mode_sleep.cpp
	src/mode/mode_charge.cpp
	src/mode/mode_idle.cpp
	src/mode/mode_remote.cpp
	src/mode/mode_go_to_charger.cpp
	src/mode/clean_mode_exploration.cpp
	src/mode/clean_mode.cpp
	src/mode/clean_mode_navigation.cpp
	src/mode/clean_mode_follow_wall.cpp
	src/mode/clean_mode_spot.cpp
	src/mode/clean_mode_test.cpp

	src/path_algorithm/path_algorithm.cpp
	src/path_algorithm/path_algorithm_navigation_clean.cpp
	src/path_algorithm/path_algorithm_follow_wall.cpp
	src/path_algorithm/path_algorithm_spot.cpp
	src/path_algorithm/path_algorithm_go_home.cpp

	src/speed_governor/*

	src/action/action.cpp
	src/action/action_open_gyro.cpp
	src/action/action_back_from_charger.cpp
	src/action/action_open_lidar.cpp
	src/action/action_align.cpp
	src/action/action_open_slam.cpp
	src/action/action_sleep.cpp
	src/action/action_idle.cpp
	src/action/action_pause.cpp
	src/action/action_check_vacuum.cpp
	src/action/action_check_bumper.cpp
	src/action/action_check_water_tank.cpp
	src/action/action_life_check.cpp
	src/action/action_r16_test.cpp
	src/action/action_open_gyro_and_lidar.cpp

	src/action/move_type/move_type.cpp
	src/action/move_type/move_type_linear.cpp
	src/action/move_type/move_type_follow_wall.cpp
	src/action/move_type/move_type_go_to_charger.cpp
	src/action/move_type/move_type_bumper_hit_test.cpp
	src/action/move_type/move_type_remote.cpp
	src/action/move_type/move_type_desk_test.cpp
	src/action/move_type/move_type_gyro_test.cpp
	src/action/move_type/movement/movement.cpp
	src/action/move_type/movement/movement.cpp
	src/action/move_type/movement/movement_back.cpp
	src/action/move_type/movement/movement_rcon.cpp
	src/action/move_type/movement/movement_follow_wall_infrared.cpp
	src/action/move_type/movement/movement_follow_wall_lidar.cpp
	src/action/move_type/movement/movement_follow_point_linear.cpp
	src/action/move_type/movement/movement_go_charger.cpp
	src/action/move_type/movement/movement_exception_resume.cpp
	src/action/move_type/movement/movement_turn.cpp
	src/action/move_type/movement/movement_turn.cpp
	src/action/move_type/movement/movement_charge.cpp
	src/action/move_type/movement/movement_stay.cpp
	src/action/move_type/movement/movement_direct_go.cpp
	src/action/move_type/movement/movement_straight.cpp
	src/action/move_type/movement/movement_forward_turn.cpp
	src/action/move_type/movement/movement_follow_point.cpp
	src/action/move_type/movement/movement_gyro_dynamic.cpp

	src/des.cpp
	src/error.cpp
	src/event_manager.cpp
	src/main.cpp
	src/map.cpp
	src/mathematics.cpp
	src/odom.cpp
	src/pose.cpp
	src/x900_functional_test.cpp
	src/robot.cpp
	src/serial.cpp
	src/signature.c
	src/slam.cpp
	src/slam_map.cpp
	src/verify.cpp

	src/dev/wifi/dev.cpp
	src/dev/wifi/packet.cpp
	src/dev/wifi/tx.cpp
	src/dev/wifi/rx.cpp
	src/dev/wifi/msg.cpp
	#src/wifi/util.cpp

	src/protocol/wifi_map_protocol.cpp

	#src/cloud/realtime_map_handler.cpp
	#src/cloud/accumulated_work_time_handler.cpp
	#src/cloud/consumable_handler.cpp
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
include_directories(/opt/projects/yocto/yocto-build/tmp/work/cortexa7hf-neon-vfpv4-poky-linux-gnueabi/rplidar-ros/git-r0/build/devel/include)
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
	x900ctrl.msg
	x900sensor.msg
)

#add_service_files(
#	DIRECTORY srv
#	FILES
#)

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

add_executable(pp  ${PP_SRC} )
#target_link_libraries(pp ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${Eigen3_LIBRARIES} -lm -lpthread)
target_link_libraries(pp ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${Eigen3_LIBRARIES} -lm -lpthread "-T ../ilife_ldscript.lds ../res.o -Wl,--verbose")

add_executable(syscheck src/signature.c src/des.cpp src/syscheck.cpp)
#target_link_libraries(syscheck ${catkin_LIBRARIES} -lm "-T ../ilife_ldscript.lds -Wl,--verbose")
target_link_libraries(syscheck ${catkin_LIBRARIES})
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

install(DIRECTORY launch urdf config audio lidar_checking
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES custom_rosconsole.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
