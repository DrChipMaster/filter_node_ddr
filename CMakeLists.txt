cmake_minimum_required(VERSION 3.5)
project(filter_node)
#if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
#  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
#  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
#  set(CMAKE_CXX_STANDARD 11)
#endif()
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_NO_SYSTEM_FROM_IMPORTED ON) # UI files
set(CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES ON) # UI files
set(CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES ON) # UI files
include_directories(include ${catkin_INCLUDE_DIRS})


#set(CMAKE_NO_SYSTEM_FROM_IMPORTED ON)‍

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()
#find_package(QT NAMES Qt5 COMPONENTS Widgets REQUIRED)
#find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_filters
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS filesystem thread)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nodelet
    nodelet_topic_tools
    pcl_conversions
    pcl_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    PCL
    message_runtime
)



add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp filters.cpp filters.h rosthread.cpp rosthread.h)

add_executable(filter_node ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS filter_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS filter_node
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)