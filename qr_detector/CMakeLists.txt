cmake_minimum_required(VERSION 2.8.11)
project(qr_detector)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
             rospy roscpp nodelet sensor_msgs std_msgs cv_bridge image_transport)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qr_detector_nodelet
  CATKIN_DEPENDS roscpp rospy nodelet conf_move sensor_msgs cv_bridge image_transport std_msgs
)

find_library(ZBAR_LIBRARIES NAMES zbar)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(qr_detector_nodelet src/qr_detector_nodelet.cpp src/detector.cpp scripts/conf_move.py)
target_link_libraries(qr_detector_nodelet zbar ${catkin_LIBRARIES})

install(TARGETS qr_detector_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
   find_package(roslaunch REQUIRED)
   roslaunch_add_file_check(launch)
endif()
