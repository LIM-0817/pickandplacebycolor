#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 이 노드는 joint_state_publishser_gui가 발행하는 joint_states라는 토픽을 (sensor_msgs/msg/JointState 형식)
# arm_controller과 gripper_controller이 받는 arm_controller/JointTrajectory와 
# gripper_controller/JointTrajectory (trajectory_msgs/msg/JointTrajectory 형식)으로 보내주는 거임.
# 즉 joint_state_publishser_gui의 슬라이더로 관절을 움직일 때 로봇이 가제보 상에서 ros2 control을 통해 움직이도록 만듦

class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()
        
        arm_controller.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        gripper_controller.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        
        # JointTrajactory형식의 points는 JointTrajectoryPoint 형식의 리스트로 구성됨. 
        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()
        
        # 구독한 joint_states 토픽 msg의 position 리스트에서 첫번째-7번째까지(=1-7번 joint까지의 위치 정보)는 arm_goal에, 
        # 나머지 position은 gripper goal에 넣음
        arm_goal.positions = msg.position[:7]
        gripper_goal.positions = msg.position[7:] 
        
        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)
        
        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)

def main():
    rclpy.init()
    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()