# Robotics Project: color recognition -> pick and place 

<img width="1694" height="1085" alt="Image" src="https://github.com/user-attachments/assets/6d27fc5f-bcf1-41e4-b9ef-d22fc19f6701" />

## 초기 세팅
``` bash
colcon build --symlink-install 
```

``` bash
source ./install/setup.bash 
```

## gazebo, moveit, rviz2 등 실행
``` bash
ros2 launch panda_bringup bringup.launch.xml 
```

## pick and place
- Red
``` bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
```
- Green
``` bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
```
- Blue
``` bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B
```
