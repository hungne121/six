cmake_minimum_required(VERSION 2.8.3)
project(six)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  move_base_msgs
  actionlib
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  move_base_msgs
  actionlib
  geometry_msgs
  nav_msgs
  tf
)
catkin_install_python(PROGRAMS
  scripts/test.py
  scripts/test_1.py
  scripts/test_2.py
  scripts/test_3.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir config launch meshes urdf scripts)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY launch models worlds scripts param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)