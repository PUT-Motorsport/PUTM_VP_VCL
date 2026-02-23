cmake_minimum_required(VERSION 3.8)

project(putm_vcl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(putm_vcl_interfaces REQUIRED)

# --- INTEGRACJA NOWEJ BIBLIOTEKI CAN ---
# Wymuszamy backend ROS2 przed dodaniem podkatalogu
set(PUTM_CAN_BACKEND "ROS2" CACHE STRING "Force ROS2 Backend" FORCE)
add_subdirectory(include/PUTM_EV_CAN_LIBRARY)

add_executable(can_tx_node src/can_nodes/can_tx_node.cpp)
add_executable(can_rx_node src/can_nodes/can_rx_node.cpp)
add_executable(amk_node src/amk_node/amk_node.cpp)
add_executable(rtd_node src/rtd_node/rtd_node.cpp)
#ja
add_executable(rtd_sub_node src/rtd_sub_node/rtd_sub_node.cpp)


target_include_directories(
  amk_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
target_include_directories(
  can_tx_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_include_directories(
  can_rx_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_include_directories(
  rtd_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
                  #nwm czy tutaj cos

ament_target_dependencies(can_tx_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(can_rx_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(amk_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(rtd_node rclcpp putm_vcl_interfaces)
ament_target_dependencies(rtd_sub_node rclcpp putm_vcl_interfaces)


# --- LINKOWANIE ---
# Musisz zlinkować nową bibliotekę 'putm_ev_can' z węzłami, które jej używają
target_link_libraries(can_tx_node PRIVATE putm_ev_can)
target_link_libraries(can_rx_node PRIVATE putm_ev_can)

install(TARGETS amk_node can_tx_node can_rx_node rtd_node rtd_sub_node
        DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
