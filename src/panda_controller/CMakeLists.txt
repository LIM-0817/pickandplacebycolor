cmake_minimum_required(VERSION 3.8)
project(panda_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

# launch, config install하기
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# ament_cmake_python에서 python 파일들도 cmake에서 설치 할수 있게 해줌.
# 이걸 해줘야 python파일을 ros2 run이 가능해짐.
ament_python_install_package(${PROJECT_NAME})

# 파이썬 파일을 설치하기 위한 경로를 설정. setup.py 없이도 cmakelist.txt에 이것만 써줘도 .py파일 실행 가능
# project_name = panda_controller이므로 panda_controller안의 ~.py파일을 실행 파일로 설치하겠다는 의미
install(
  PROGRAMS ${PROJECT_NAME}/slider_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
