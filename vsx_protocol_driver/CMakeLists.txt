cmake_minimum_required(VERSION 3.0.2)
project(vsx_protocol_driver)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PF.VsxProtocolDriver.WrapperNE
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install all cmake files.
install(
  DIRECTORY lib/linux-x64/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
