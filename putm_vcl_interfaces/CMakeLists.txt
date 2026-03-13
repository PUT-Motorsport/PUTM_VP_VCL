cmake_minimum_required(VERSION 3.8)
project(putm_vcl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Amk/AmkSetpoints/AmkControl.msg"
  "msg/Amk/AmkSetpoints/AmkSetpoints.msg"
  "msg/Amk/AmkActualValues1/AmkStatus.msg"
  "msg/Amk/AmkActualValues1/AmkActualValues1.msg"
  "msg/Amk/AmkActualValues2.msg"
  "msg/Frontbox/FrontboxDriverInput.msg"
  "msg/Frontbox/FrontboxData.msg"
  "msg/Setpoints/Setpoints.msg"
  "msg/Setpoints/Setpoint.msg"
  "msg/Dashboard.msg"
  "msg/Rtd.msg"
  "msg/Xsens/XsensAcceleration.msg"
  "msg/Xsens/XsensAccelerationHr.msg"
  "msg/Xsens/XsensAlltitudeEllipsoid.msg"
  "msg/Xsens/XsensDeltaQ.msg"
  "msg/Xsens/XsensErrorCode.msg"
  "msg/Xsens/XsensEuler.msg"
  "msg/Xsens/XsensFreeAcceleration.msg"
  "msg/Xsens/XsensInertialData.msg"
  "msg/Xsens/XsensMagneticField.msg"
  "msg/Xsens/XsensOrientation.msg"
  "msg/Xsens/XsensPosition.msg"
  "msg/Xsens/XsensRateOfTurn.msg"
  "msg/Xsens/XsensRateOfTurnHr.msg"
  "msg/Xsens/XsensStatus.msg"
  "msg/Xsens/XsensTempAndPressure.msg"
  "msg/Xsens/XsensUtc.msg"
  "msg/Xsens/XsensVelocity.msg"
  "msg/BmsHvMain/BmsHvMain.msg"
  "msg/BmsLvMain/BmsLvMain.msg"
  "msg/VpData/YawRef.msg"
  "msg/PduData.msg"      
  "msg/PduChannel.msg"    
  "msg/LapTimer.msg")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
