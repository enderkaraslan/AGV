cmake_minimum_required(VERSION 3.16)

project(agv)

# C++17 standardını kullan
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

# Kaynak dosyalarını ve başlık dosyalarını doğru şekilde dahil edin
include_directories(include)

add_executable(agv_node
  src/main.cpp
  src/CameraNode.cpp
  src/LidarNode.cpp
  src/ImageProcessor.cpp
  src/Driver.cpp
  src/MotorNode.cpp
  src/Timer.cpp
)

# Hedefe bağımlılıkları ekleyin
ament_target_dependencies(agv_node
  rclcpp
  sensor_msgs
  nav_msgs
  OpenCV
  cv_bridge
  geometry_msgs
  my_robot_interfaces
  geometry_msgs
  visualization_msgs

)

# Kurulum ayarları
install(TARGETS
  agv_node
  DESTINATION lib/${PROJECT_NAME}
)

# ROS 2 paketi yapılandırması
ament_package()