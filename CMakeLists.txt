cmake_minimum_required(VERSION 3.0.2)
project(cras_joy_tools)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  nodes/joy_translator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  joy_mappings
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/joy_mappings
)
