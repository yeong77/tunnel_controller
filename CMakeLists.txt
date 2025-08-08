cmake_minimum_required(VERSION 3.8)
project(tunnel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PahoMqttCpp REQUIRED)

set(DEPS
  rclcpp
  sensor_msgs
  PahoMqttCpp
  )
 

add_executable(rover_central_node
  src/rover_central_node.cpp)
ament_target_dependencies(rover_central_node ${DEPS})

target_link_libraries(rover_central_node
  PahoMqttCpp::paho-mqttpp3
  jsoncpp
)


install(TARGETS
  rover_central_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
