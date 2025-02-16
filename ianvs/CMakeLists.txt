cmake_minimum_required(VERSION 3.22.1)
project(ianvs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/image_publisher.cpp src/image_subscription.cpp src/node_handle.cpp
  src/node_handle_factory.cpp src/spin_functions.cpp
)
add_library(ianvs::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME} PUBLIC message_filters::message_filters rclcpp::rclcpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "${PROJECT_NAME}_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME} PUBLIC sensor_msgs std_srvs)
if(NOT BUILD_SHARED_LIBS)
  set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-exports
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}-exports HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs std_srvs)
ament_package()
