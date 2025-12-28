# Panda robot이 색을 인식해서 pick and place

## 초기 세팅
''' colcon build --symlink-install''' bash
''' source ./install/setup.bash''' bash

## gazebo, moveit, rviz2 등 실행
''' ros2 launch panda_bringup bringup.launch.xml ''' bash

## pick and place
- Red
''' ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R ''' bash

- Green
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G ''' bash

- Blue
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B ''' bash
