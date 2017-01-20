cmake_minimum_required(VERSION 2.8.3)
project(pp)

FILE(GLOB PP_SRC
	"./src/*.c"
	"./src/*.cpp"
)
add_message_files(
	DIRECTORY msg
	FILES
	x900sensor.msg
	slam_angle_offset.msg
	robotmode.msg
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
	tf
)
generate_messages(
	DEPENDENCIES
	std_msgs
	visualization_msgs
	geometry_msgs
	sensor_msgs
)




include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
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
)

add_executable(pp  ${PP_SRC} )
target_link_libraries(pp ${catkin_LIBRARIES} -lm -lpthread)

install(TARGETS pp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
