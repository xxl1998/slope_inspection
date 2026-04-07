cmake_minimum_required(VERSION 3.0.2)
project(so3_quadrotor_sim)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  quadrotor_msgs
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS quadrotor_msgs roscpp tf
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(so3_quadrotor
  src/so3_quadrotor.cpp
)

target_link_libraries(so3_quadrotor
  ${catkin_LIBRARIES}
)

add_dependencies(so3_quadrotor
  ${catkin_EXPORTED_TARGETS}
  quadrotor_msgs_generate_messages_cpp
)


add_library(so3_controller
  src/so3_controller.cpp
)

target_link_libraries(so3_controller
  ${catkin_LIBRARIES}
)

add_dependencies(so3_controller
  ${catkin_EXPORTED_TARGETS}
  quadrotor_msgs_generate_messages_cpp
)

add_executable(so3_quadrotor_sim_node
  src/so3_quadrotor_sim.cpp
)
target_link_libraries(so3_quadrotor_sim_node
  ${catkin_LIBRARIES}
)
add_dependencies(so3_quadrotor_sim_node
  ${catkin_EXPORTED_TARGETS}
  quadrotor_msgs_generate_messages_cpp
)
