cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImgDetectionData.msg"
  "msg/ApproachTargetInfo.msg"
  "msg/TrackedBbox.msg"
  "srv/SelectObject.srv"
  "srv/ApproachObject.srv"
  "srv/ArduinoController.srv"
  "srv/QLearning.srv"
  "srv/BboxToXYZ.srv"
  "srv/GetObjProperties.srv"
  "srv/DetectObjectsV2.srv"
  "srv/StartSearch.srv"
  "srv/StartGrasp.srv"
  "srv/SetJointState.srv"
  "srv/MoveJoint.srv"
  "srv/MoveEndEffectorCount.srv"
  "srv/GetEndEffectorCount.srv"
  "action/XArm.action"
  "action/ApproachObject.action"
  "action/WskrSearch.action"
  "action/PlayWaypointsDense.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
