#!/usr/bin/env python

"""
Node to convert joystick commands to kinova arm cartesian movements
"""

import rospy
from sensor_msgs.msg import Joy
from kortex_driver.msg import TwistCommand, Gripper, Finger
from kortex_driver.srv import SendGripperCommand, SendGripperCommandRequest

max_linear_speed = 0.1
max_angular_speed = 0.4
gripper_speed = 0.05

def joy_listener():

    # start node
    rospy.init_node("kinova_joy_teleop")

    # subscribe to joystick messages on topic "joy"
    rospy.Subscriber("/joy_teleop/joy", Joy, joy_cmd_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()

def joy_cmd_callback(data):

    # start publisher
    pub = rospy.Publisher("in/cartesian_velocity", TwistCommand, queue_size=1)

    # create gripper command message
    cmd = TwistCommand()
    if (data.axes[2] < 0):
        cmd.twist.linear_x = data.axes[1] * max_linear_speed
        cmd.twist.linear_y = data.axes[0] * max_linear_speed
        cmd.twist.linear_z = data.axes[4] * max_linear_speed
        cmd.twist.angular_z = -data.axes[3] * max_angular_speed
        rospy.loginfo("linear velocities: {%f, %f, %f};", cmd.twist.linear_x, cmd.twist.linear_y, cmd.twist.linear_z) 
    elif (data.axes[5] < 0):
        cmd.twist.angular_x = data.axes[1] * max_angular_speed
        cmd.twist.angular_y = data.axes[0] * max_angular_speed
        cmd.twist.angular_z = -data.axes[3] * max_angular_speed
        rospy.loginfo("angular velocities: {%f, %f, %f};", cmd.twist.angular_x, cmd.twist.angular_y, cmd.twist.angular_z)
    
    if (data.buttons[0] == 1 or data.buttons[1] == 1):
        cmd_gripper_req = SendGripperCommandRequest()
        cmd_gripper_req.input.mode = 2
        fingey = Finger()
        gripper_dir = -1 if data.buttons[0] == 1 else 1
        fingey.value = gripper_dir*gripper_speed
        cmd_gripper_req.input.gripper.finger.append(fingey)
        try:
            cmd_gripper_srv = rospy.ServiceProxy('base/send_gripper_command', SendGripperCommand)
            cmd_gripper_srv(cmd_gripper_req)
        except rospy.ServiceException as e:
            rospy.loginfo(cmd_gripper_req)
            rospy.loginfo("joystick gripper command failed")

        

    # publish gripper command
    pub.publish(cmd)

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
