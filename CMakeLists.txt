cmake_minimum_required(VERSION 2.8.3)
project(robor_controllers)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS tf roscpp roslaunch nodelet rviz joy std_srvs nav_msgs geometry_msgs obstacle_detector)
find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manual_controller feedback_controller potentials_controller #urakubo_controller 
  CATKIN_DEPENDS tf roscpp nodelet rviz joy std_srvs nav_msgs geometry_msgs obstacle_detector
)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

#
# Build libs
#
add_library(manual_controller src/manual_controller.cpp)
target_link_libraries(manual_controller ${roscpp_LIBRARIES})
add_dependencies(manual_controller ${catkin_EXPORTED_TARGETS})

add_library(feedback_controller src/feedback_controller.cpp)
target_link_libraries(feedback_controller ${roscpp_LIBRARIES})
add_dependencies(feedback_controller ${catkin_EXPORTED_TARGETS})

#add_library(urakubo_controller src/urakubo_controller.cpp)
#target_link_libraries(urakubo_controller ${roscpp_LIBRARIES} ${ARMADILLO_LIBRARIES})
#add_dependencies(urakubo_controller ${catkin_EXPORTED_TARGETS})

add_library(potentials_controller src/potentials_controller.cpp)
target_link_libraries(potentials_controller ${roscpp_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(potentials_controller ${catkin_EXPORTED_TARGETS})

#
# Build nodes
#
add_executable(manual_controller_node src/nodes/manual_controller_node.cpp)
target_link_libraries(manual_controller_node manual_controller)

add_executable(feedback_controller_node src/nodes/feedback_controller_node.cpp)
target_link_libraries(feedback_controller_node feedback_controller)

#add_executable(urakubo_controller_node src/nodes/urakubo_controller_node.cpp)
#target_link_libraries(urakubo_controller_node urakubo_controller)

add_executable(potentials_controller_node src/nodes/potentials_controller_node.cpp)
target_link_libraries(potentials_controller_node potentials_controller)

#
# Build nodelets
#
add_library(${PROJECT_NAME}_nodelets
  src/nodelets/manual_controller_nodelet.cpp
  src/nodelets/feedback_controller_nodelet.cpp
  #src/nodelets/urakubo_controller_nodelet.cpp
  src/nodelets/potentials_controller_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets manual_controller feedback_controller potentials_controller) #urakubo_controller 

#
# Build rviz plugins
#
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}_gui
  src/panels/manual_controller_panel.cpp include/${PROJECT_NAME}/panels/manual_controller_panel.h
  src/panels/feedback_controller_panel.cpp include/${PROJECT_NAME}/panels/feedback_controller_panel.h
  #src/panels/urakubo_controller_panel.cpp include/${PROJECT_NAME}/panels/urakubo_controller_panel.h
  src/panels/potentials_controller_panel.cpp include/${PROJECT_NAME}/panels/potentials_controller_panel.h)
target_link_libraries(${PROJECT_NAME}_gui ${roscpp_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})

#
# Install libraries
#
install(TARGETS manual_controller feedback_controller potentials_controller ${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gui #urakubo_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#
# Install nodes
#
install(TARGETS manual_controller_node feedback_controller_node potentials_controller_node #urakubo_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# Install header files
#
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#
# Install nodelet and rviz plugins description
#
install(FILES nodelet_plugins.xml rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Install launch files
#
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Create folders and copy resources
#
file(MAKE_DIRECTORY $ENV{HOME}/.local/share/icons/robor)
file(COPY resources/play.png resources/pause.png resources/stop.png DESTINATION $ENV{HOME}/.local/share/icons/robor)
