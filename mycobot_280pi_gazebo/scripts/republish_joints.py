#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReorderer(Node):
    def __init__(self):
        super().__init__('joint_state_reorderer')
        

        # Subscriber to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Publisher for reordered joint states
        self.publisher = self.create_publisher(JointState, '/ordered_joint_states', 10)

        self.get_logger().info("Republishing Node started!")  # Log info message


        # Desired order of joints
        self.ordered_joints = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
            'gripper_controller',
            'gripper_base_to_gripper_left2',
            'gripper_left3_to_gripper_left1',
            'gripper_base_to_gripper_right3',
            'gripper_base_to_gripper_right2',
            'gripper_right3_to_gripper_right1'            
        ]
        
    def joint_state_callback(self, msg):
        # Reorder the joint states
        reordered_msg = JointState()
        reordered_msg.header = msg.header  # Preserve header
        
        # Ensure joints are ordered as per the desired list
        for joint_name in self.ordered_joints:
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                reordered_msg.name.append(joint_name)
                reordered_msg.position.append(msg.position[index])
                #reordered_msg.velocity.append(msg.velocity[index])
                #reordered_msg.effort.append(msg.effort[index])
        
        # Publish the reordered message
        self.publisher.publish(reordered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateReorderer()

    
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
