# Panda robot이 색을 인식해서 pick and place

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
