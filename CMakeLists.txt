##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ros_gui)
add_compile_options(-std=c++11)
##############################################################################
# Catkin
##############################################################################

set(BOOST_ROOT /home/pyzhu/boost_1_58_0)
#set(Boost_LIBRARY_DIRS /home/pyzhu/boost_1_58_0/stage/lib)
#set(CMAKE_CXX_FLAGS "-lboost_system")

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp geometry_msgs message_generation tf moveit_ros_planning_interface)
find_package(Boost REQUIRED COMPONENTS system filesystem)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ARMarker.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LocalizePart.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs  # Or other packages containing msgs
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myworkcell_core
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_msgs
    message_generation
    tf
    moveit_ros_planning_interface
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/ros_gui/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
#link_directories(${Boost_LIBRARY_DIRS})
add_executable(ros_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(vision_node server/vision_node.cpp)
target_link_libraries(ros_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(vision_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ros_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

