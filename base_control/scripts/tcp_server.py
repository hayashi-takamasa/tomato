#!/usr/bin/env python3
# -*- coding : UTF-8 -*-

import socket
import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi


class TCP_Server():
    def __init__(self):
        server_ip = "127.0.0.1"
        server_port = 8080
        listen_num = 5
        self.buffer_size = 1024
        self.odom = 0.0
        self.destination = [1.8, 0.0, 0.0]
        self.stop_msg = Twist()
        
        print("TCP_server start")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((server_ip, server_port))
        self.server.listen(listen_num)
        print("Movebase Action Client start")
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.loginfo("The robot was terminated.")                               
        self.ac.cancel_goal()

    def odomCallback(self, msg):
        self.odom = msg.pose.position.x

    def send_stop(self):
        self.stop_msg.linear.y = 0
        self.stop_msg.linear.z = 0
        self.stop_msg.angular.x = 0
        self.stop_msg.angular.y = 0
        self.stop_msg.angular.z = 0
        self.stop_vel_pub.publish(self.stop_msg)

    def send_goal(self):
        self.goal.target_pose.header.frame_id = 't265_odom_frame'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = self.destination[0]
        self.goal.target_pose.pose.position.y = self.destination[1]
        q = tf.transformations.quaternion_from_euler(0, 0, self.destination[2])
        self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        # rospy.loginfo("Current Location : " + self.odom.pose.position.x)
        # rospy.loginfo("Sending goal : " + destination)
        self.ac.send_goal(self.goal)
        succeeded = self.ac.wait_for_result(rospy.Duration(1))
        state = self.ac.get_state()

        print(state)
        if succeeded:
            rospy.loginfo("Arrived destination")
        else:
            rospy.loginfo("Not arrvied destination")
        rospy.sleep(2)

    def process(self):
        rospy.Subscriber("/t265/odom/sample", Odometry, self.odomCallback)
        self.stop_vel_pub = rospy.Publisher("/base_velocity_controller/cmd_vel", Twist, queue_size=1)

        while not rospy.is_shutdown():
            client,address = self.server.accept()
            print("------------------------------------")
            print("[*] Connected!! [ Source : {}]".format(address))
            data = client.recv(self. buffer_size)
            print("[*] Received Data : {}".format(data))
            if(data == b"Go"):
                print("Sending Goal")
                self.send_goal()
            if(data == b"Stop"):
                print("Cancel destination and Sending Stop")
                self.ac.cancel_goal()
                self.send_stop()

            client.send(b"Received by dolly")
            client.close()


if __name__ == '__main__':
    try:
        rospy.init_node('TCP_server', anonymous=True)
        tcp_server = TCP_Server()
        tcp_server.process()
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TCP_server is finished.")

        