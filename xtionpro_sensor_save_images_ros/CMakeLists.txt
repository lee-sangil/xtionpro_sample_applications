SET(PROJECT_NAME xtionpro_sensor_save_images_ros)
PROJEcT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)


# Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge roslib image_transport sensor_msgs std_msgs geometry_msgs message_runtime message_filters)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system filesystem thread REQUIRED)


## Declare a catkin package
catkin_package()


# Set libraries
LIST(APPEND LINK_LIBS
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)


# Include dirs
INCLUDE_DIRECTORIES(
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)


# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "-msse3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -O2 -flto -fopenmp -march=native")


################################################################################
# Executable files
ADD_EXECUTABLE(${PROJECT_NAME} src/main_script.cpp ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES} ${LINK_LIBS})

