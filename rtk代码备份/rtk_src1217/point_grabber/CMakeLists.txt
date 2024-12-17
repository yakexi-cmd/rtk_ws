cmake_minimum_required(VERSION 3.0.2)
project(point_grabber)

set(CMAKE_AUTOMOC ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_msgs
)

if(rviz_QT_VERSION VERSION_LESS "5")
     message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
     find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
     include(${QT_USE_FILE})
else()
     message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
     find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
     set(QT_LIBRARIES Qt5::Widgets)
endif()
 

# 处理 moc
foreach(SOURCE ${SOURCES})
  get_filename_component(SOURCE_NAME ${SOURCE} NAME_WE)
  set(MOC_FILE ${CMAKE_CURRENT_BINARY_DIR}/moc_${SOURCE_NAME}.cpp)
  add_custom_command(
    OUTPUT ${MOC_FILE}
    COMMAND ${QT5_MOC_EXECUTABLE}
    ARGS -o ${MOC_FILE} ${SOURCE}
    DEPENDS ${SOURCE}
  )
  list(APPEND MOC_SOURCES ${MOC_FILE})
endforeach()
 
catkin_package(
     CATKIN_DEPENDS rviz std_msgs 
     DEPENDS rviz
)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
 
add_library(${PROJECT_NAME} 
     src/point_grabber.cpp
     ${UIC_FILES}
     )
 
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
