cmake_minimum_required(VERSION 3.22.1)
project(ianvs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(CLI11 REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

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

add_executable(play_rosbag app/play_rosbag.cpp)
target_link_libraries(
  play_rosbag PRIVATE rclcpp::rclcpp rosbag2_transport::rosbag2_transport Boost::system
                      Boost::filesystem CLI11::CLI11
)
ament_target_dependencies(play_rosbag PUBLIC tf2_msgs tf2_ros)

pybind11_add_module(
  _${PROJECT_NAME}_bindings src/bindings/image_encodings.cpp src/bindings/module.cpp
)
target_include_directories(
  _${PROJECT_NAME}_bindings
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(_${PROJECT_NAME}_bindings PUBLIC sensor_msgs)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)
install(
  TARGETS ${PROJECT_NAME} play_rosbag
  EXPORT ${PROJECT_NAME}-exports
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_get_python_install_dir(PYTHON_INSTALL_PATH)
install(TARGETS _${PROJECT_NAME}_bindings
        ARCHIVE DESTINATION ${PYTHON_INSTALL_PATH}/ianvs
        LIBRARY DESTINATION ${PYTHON_INSTALL_PATH}/ianvs
)
ament_python_install_package(${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-exports HAS_LIBRARY_TARGET)
ament_export_dependencies(message_filters rclcpp sensor_msgs std_srvs)
ament_package()
