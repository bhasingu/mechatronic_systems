cmake_minimum_required(VERSION 2.8)
project(mapping)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${OpenCV_LIB})