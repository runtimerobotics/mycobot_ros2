import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math
import sys



class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "ordered_joint_states",
            self.listener_callback,
            10
        )
        self.subscription

        self.enable_hw=int(sys.argv[1])

        if(self.enable_hw):
            self.mc = MyCobot("/dev/ttyAMA0", 1000000)
            print("Initialize control node and robot")
            time.sleep(0.05)
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)
        self.prev_grip_val=0

    def scale_value(self,value, input_min, input_max, output_min, output_max):
    # Apply the scaling formula
        return ((value - input_min) / (input_max - input_min)) * (output_max - output_min) + output_min



    def listener_callback(self, msg):

        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
            
        #print('data_list: {}'.format(data_list))

        arm_joints=data_list[:-1]
        print("Arm joints:",arm_joints)

        if(self.enable_hw):
            if(not self.mc.is_gripper_moving()):
                self.mc.send_angles(arm_joints, 25)
	#Sending to gripper
        gripper_value=data_list[-1:]
	
        #print( "Gripper value:",gripper_value)

        input_min = -34
        input_max = 8.6
        output_min = 0
        output_max = 100
        # Example value to scale
        value = gripper_value[0]
        grip_final = self.scale_value(value, input_min, input_max, output_min, output_max)
        #print(grip_final)  # Output will be between 0 and 100

        self.current_grip_val=int(grip_final)
        print("Current gripper value",self.current_grip_val)

        if(self.enable_hw):
            if(self.current_grip_val != self.prev_grip_val):
                    if(not self.mc.is_gripper_moving()):
                        print("Sending to gripper:",self.current_grip_val)
                        self.mc.set_gripper_value(self.current_grip_val,70,1)
                #self.mc.set_gripper_state(1,70);
        #self.mc.set_gripper_value(self.current_grip_val,20,1)

        self.prev_grip_val = self.current_grip_val

def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
