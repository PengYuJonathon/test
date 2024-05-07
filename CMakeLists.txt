cmake_minimum_required(VERSION 3.5)
project(trackbar)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)


add_executable(talker src/pub.cpp)
target_link_libraries(talker ${OpenCV_LIBS})

add_executable(listener src/sub.cpp)
target_link_libraries(listener ${OpenCV_LIBS})


ament_target_dependencies(
  talker
  "rclcpp"
  "OpenCV"
  "cv_bridge"
  "sensor_msgs"
  "image_transport"
)

ament_target_dependencies(
  listener
  "rclcpp"
  "OpenCV"
  "cv_bridge"
  "sensor_msgs"
  "image_transport"
)

install(TARGETS talker listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()
