import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# urdf에 들어가는 arguments들 정의
ARGUMENTS = [
    DeclareLaunchArgument('is_sim', default_value='true',choices=['true', 'false']),
    DeclareLaunchArgument('is_ignition', default_value='false', choices=['true', 'false'])
            ]

def generate_launch_description():
    # urdf.xacro 경로 지정
    description_pkg = get_package_share_directory('panda_description') 
    urdf_path = os.path.join(description_pkg, 'urdf', 'panda.urdf.xacro')   # moveit_config_builder 내부 경로는 os.path.join으로!

    # moveit_config 빌딩하기
    moveit_config = (MoveItConfigsBuilder(
        'panda', 
        package_name='panda_moveit')
        .robot_description(
            file_path=urdf_path,
            mappings={'is_ignition': LaunchConfiguration('is_ignition')}  # xacro의 is_ignition 자리에 Argument에서 정의한 is_ignition 대입 
            )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs() 
    )

    # moveit 노드 선언하기. moveit의 노드는 경로 계획 및 실행, 로봇 상태와 주변 환경에 대한 센서 정보를 받음
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),                            # 위에서 정의한 moveit_config를 파라미터로 넣어줌. 이때, to_dict로 dict형식으로 바꿔서 넣어줘야 함.
            {"use_sim_time": LaunchConfiguration("is_sim")},    # use_sim_time이라는 파라미터에 'is_sim' 대입
            {"start_state_max_bounds_error": 0.1},              # 하도 관절 각이 벗어났다고 난리 쳐서 0.1 정도의 오차는 봐주라고 함.
            {"publish_robot_description_semantic": True}        # srdf를 토픽으로 발행해 rviz 등의 노드가 받아볼 수 있게 할건지 말건지 결정
        ],
        arguments=["--ros-args", "--log-level", "info"], # ros2의 log 수준을 info로 하겠다는 arguments
    )

    # RViz config파일 경로
    rviz_config = os.path.join(
        get_package_share_directory("panda_moveit"),
        "rviz",
        "moveit.rviz",
    )

    # rviz 노드 선언
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        
        # rviz의 파라미터에 moveit_config의 파라미터들을 전달
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("is_sim")}
        ],
    )

    ls = LaunchDescription(ARGUMENTS)
    ls.add_action(move_group_node)
    ls.add_action(rviz_node)

    return ls