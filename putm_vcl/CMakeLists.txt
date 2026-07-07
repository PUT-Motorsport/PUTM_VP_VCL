cmake_minimum_required(VERSION 3.8)

project(putm_vcl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(putm_vcl_interfaces REQUIRED)

#add_executable(can_tx_node src/can_nodes/can_tx_node.cpp)
#add_executable(can_rx_node src/can_nodes/can_rx_node.cpp)
add_executable(amk_node src/amk_node/amk_node.cpp)
add_executable(rtd_node src/rtd_node/rtd_node.cpp)
#ja
add_executable(rtd_sub_node src/rtd_sub_node/rtd_sub_node.cpp)
add_executable(rec_node src/rec_node/rec_node.cpp)

target_include_directories(
  amk_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
# target_include_directories(
#   can_tx_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#                      $<INSTALL_INTERFACE:include>)
# target_include_directories(
#   can_rx_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#                      $<INSTALL_INTERFACE:include>)
target_include_directories(
  rtd_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
                  #nwm czy tutaj cos
target_include_directories(
  rec_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)                  

# ament_target_dependencies(can_tx_node rclcpp putm_vcl_interfaces)
# ament_target_dependencies(can_rx_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(amk_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(rtd_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(rtd_sub_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(rec_node rclcpp putm_vcl_interfaces)

install(TARGETS amk_node rtd_node rtd_sub_node rec_node
        DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
