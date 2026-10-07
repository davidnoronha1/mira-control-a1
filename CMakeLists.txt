cmake_minimum_required(VERSION 3.8)
project(mira2_control_master_go NONE)

find_package(ament_cmake REQUIRED)
# Only needed so colcon builds (and sources) these before running rclgo-gen.
find_package(custom_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_program(GO_EXECUTABLE go REQUIRED)

set(MASTER_BIN ${CMAKE_CURRENT_BINARY_DIR}/master)
add_custom_target(master ALL
  COMMAND ${CMAKE_COMMAND} -E env "PATH=$ENV{PATH}"
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build.sh ${MASTER_BIN}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  BYPRODUCTS ${MASTER_BIN}
  COMMENT "Building Go master node"
  USES_TERMINAL)

install(PROGRAMS ${MASTER_BIN} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
