cmake_minimum_required(VERSION 3.22.1)
project(ianvs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # from: https://www.kitware.com//cmake-and-the-default-build-type/
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

if(NOT CMAKE_INTERPROCEDURAL_OPTIMIZATION)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON CACHE STRING "Turn on -flto" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(CLI11 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

# core library for use downstream
add_library(
  ${PROJECT_NAME}
  src/detail/string_transforms.cpp
  src/image_publisher.cpp
  src/image_subscription.cpp
  src/node_handle.cpp
  src/node_handle_factory.cpp
  src/node_init.cpp
  src/spin_functions.cpp
)
add_library(ianvs::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME} PUBLIC CLI11::CLI11 message_filters::message_filters rclcpp::rclcpp
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

add_library(ament_shim INTERFACE)
ament_target_dependencies(ament_shim INTERFACE tf2_msgs tf2_ros)

add_library(${PROJECT_NAME}_rosbag src/bag_reader.cpp)
add_library(ianvs::${PROJECT_NAME}_rosbag ALIAS ${PROJECT_NAME}_rosbag)
target_link_libraries(
  ${PROJECT_NAME}_rosbag PUBLIC rosbag2_transport::rosbag2_transport rclcpp::rclcpp
)
target_include_directories(
  ${PROJECT_NAME}_rosbag PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
if(NOT BUILD_SHARED_LIBS)
  set_property(TARGET ${PROJECT_NAME}_rosbag PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

# various plugins for executables to load
add_library(
  ${PROJECT_NAME}_plugins SHARED src/app/signal_stop_plugin.cpp src/app/tf_plugin.cpp
                                 src/app/topic_remap_plugin.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME} rosbag2_transport::rosbag2_transport
  PRIVATE ament_shim
)

# executables
add_executable(play_rosbag app/play_rosbag.cpp)
target_link_libraries(
  play_rosbag PRIVATE ${PROJECT_NAME} rosbag2_transport::rosbag2_transport
)

add_executable(merge_rosbags app/merge_rosbags.cpp)
target_link_libraries(
  merge_rosbags PRIVATE ${PROJECT_NAME} rosbag2_transport::rosbag2_transport
)

add_executable(transform_file_broadcaster app/transform_file_broadcaster.cpp)
ament_target_dependencies(transform_file_broadcaster PUBLIC tf2_ros rclcpp)
target_link_libraries(transform_file_broadcaster PUBLIC yaml-cpp::yaml-cpp)

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
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins ${PROJECT_NAME}_rosbag play_rosbag
          merge_rosbags transform_file_broadcaster
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

pluginlib_export_plugin_description_file(ianvs plugins.xml)
ament_export_targets(${PROJECT_NAME}-exports HAS_LIBRARY_TARGET)
ament_export_dependencies(message_filters rclcpp sensor_msgs std_srvs CLI11)
ament_package()
