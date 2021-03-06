cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

set(CMAKE_CXX_FLAGS "-O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

include_directories(/home/kimsooyoung/Documents/unitree_legged_sdk/include)
link_directories(/home/kimsooyoung/Documents/unitree_legged_sdk/lib)

################

# string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk.so)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

####################

# add_executable(lcm_server /home/kimsooyoung/Documents/unitree_legged_sdk/examples/lcm_server.cpp)
# target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(lcm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(position_mode src/exe/position_mode.cpp)
# target_link_libraries(position_mode ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(position_mode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lcm_server /home/kimsooyoung/Documents/unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})
# ament_target_dependencies(lcm_server rclcpp)

add_executable(position_mode src/position_mode.cpp)
target_link_libraries(position_mode ${EXTRA_LIBS})
ament_target_dependencies(position_mode rclcpp unitree_legged_msgs)

add_executable(walk_mode src/walk_mode.cpp)
target_link_libraries(walk_mode ${EXTRA_LIBS})
ament_target_dependencies(walk_mode rclcpp unitree_legged_msgs)

add_executable(ros_control src/ros_control.cpp)
target_link_libraries(ros_control ${EXTRA_LIBS})
ament_target_dependencies(ros_control 
  rclcpp 
  unitree_legged_msgs 
  std_msgs
  geometry_msgs 
  sensor_msgs
)

add_executable(high_msg_test src/high_msg_test.cpp)
ament_target_dependencies(high_msg_test rclcpp unitree_legged_msgs)

add_executable(msg_test src/msg_test.cpp)
ament_target_dependencies(msg_test rclcpp unitree_legged_msgs)

add_executable(imu_msg_test src/imu_msg_test.cpp)
ament_target_dependencies(imu_msg_test rclcpp unitree_legged_msgs)


install(TARGETS
   lcm_server
   position_mode
   msg_test
   imu_msg_test
   walk_mode
   ros_control
  DESTINATION 
   lib/${PROJECT_NAME}
)

ament_package()
