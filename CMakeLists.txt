cmake_minimum_required(VERSION 2.8.3)
project(pp)

FILE(GLOB PP_SRC
	"./src/*.c"
	"./src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	rosconsole
	roscpp
	sensor_msgs
	std_msgs
	tf
)

add_message_files(
	FILES
	periph.msg
	sensor.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

include_directories(
  ./include
  ./src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
	roscpp
	std_msgs
	tf
)

add_executable(pp ${PP_SRC} )
target_link_libraries(pp ${catkin_LIBRARIES} -lm -lpthread)

install(TARGETS pp 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
