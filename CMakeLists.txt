cmake_minimum_required(VERSION 3.8)
project(ffmpeg_image_transport)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

option(COMPILE_FFMPEG "Compiles FFMPEG from git with hwaccel" OFF)
if (${COMPILE_FFMPEG})
  include(cmake/BuildFFMPEG.cmake)
else()
  find_path(FFMPEG_AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
  find_library(FFMPEG_AVCODEC_LIBRARY avcodec)
  find_library(FFMPEG_SWSCALE_LIBRARY swscale)

  list(APPEND FFMPEG_INCLUDE_DIR ${FFMPEG_AVCODEC_INCLUDE_DIR})
  list(APPEND FFMPEG_LIBRARIES ${FFMPEG_AVCODEC_LIBRARY})
  list(APPEND FFMPEG_LIBRARIES ${FFMPEG_SWSCALE_LIBRARY})
endif()

include_directories(include ${FFMPEG_INCLUDE_DIR})

add_library(${PROJECT_NAME}_component SHARED
  src/ffmpeg_publisher.cpp
  src/ffmpeg_subscriber.cpp
  src/manifest.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_component
  image_transport
  rclcpp
  pluginlib
  std_msgs
)
target_link_libraries(
  ${PROJECT_NAME}_component
  ${FFMPEG_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}_component PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME}_component PRIVATE "IMAGE_TRANSPORT_BUILDING_DLL")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Packet.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
  ADD_LINTER_TESTS
)

rosidl_target_interfaces(${PROJECT_NAME}_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(image_transport ffmpeg_plugins.xml)

ament_package()
