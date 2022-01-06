#!/usr/bin/env python3
from math import trunc
import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Float64
import serial

class SerialTwist():
    def __init__(self):
        rospy.init_node("serialtwist", anonymous=True)
        rospy.Subscriber("/base_velocity_controller/cmd_vel", Twist, self.callback,queue_size=1)
        # self.ser = serial.Serial('/dev/ttyMotor', 115200)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        

    def callback(self, message):
        rospy.loginfo("subscribe cmd_vel")
        v_x = str(message.linear.x/0.95)
        v_y = str(message.linear.y)
        omega = str(message.angular.z)
        print(v_x, v_y, omega)
        self.ser.write(str.encode(v_x+','+v_y+','+omega+'\n\n'))
    

if __name__ == "__main__":
    try:
        print("Start serial connection")
        serialtwist = SerialTwist()
        rospy.spin()

    except rospy.ROSInitException:
        ser = serial.Serial('/dev/ttyUSB0', 115200)
        ser.write(str.encode('0'+','+'0'+','+'0'+'\n\n'))
        pass

