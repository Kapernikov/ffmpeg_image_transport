cmake_minimum_required(VERSION 3.5)
project(ffmpeg_image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED imgproc)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(FFMPEG REQUIRED)

# generate_dynamic_reconfigure_options(cfg/EncoderDyn.cfg)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/manifest.cpp src/ffmpeg_publisher.cpp
  src/ffmpeg_subscriber.cpp src/ffmpeg_encoder.cpp
  src/ffmpeg_decoder.cpp src/tdiff.cpp
)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${FFMPEG_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  "image_transport"
  "cv_bridge"
  "rclcpp"
  "pluginlib"
  "sensor_msgs"
  "ffmpeg_image_transport_msgs"
)

ament_export_dependencies(rosidl_default_runtime
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME})

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(image_transport ffmpeg_plugins.xml)

ament_package()

