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
import math
from math import pi
import time


class TCP_Server():
    def __init__(self):
        server_ip = "192.168.1.209"
        server_port = 8080
        listen_num = 5
        self.buffer_size = 1024
        self.odom = 0.0
        self.firstDestination = [0.95,0.0,0.0]
        self.isFirstMove = 1
        self.destinationDict = {-1:[0.95,0.0,0],1:[2.25,0.0,0]}
        #self.home_position = [0.0, 0.0, 0.0]
        #self.destination = [0.8, 0.0, 0.0]
        
        self.robot_state  = 1 #1,-1 forward backward
        self.xarm_state = "waiting"
        self.current_destination = self.destinationDict[self.robot_state]
        self.stop_msg = Twist()
        
        
        #TCP Server Initialization
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((server_ip, server_port))
        self.server.listen(listen_num)
        self.server.settimeout(None)
        print("TCP_server start")

        #Movebase Action Client initialization
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.loginfo("The robot was terminated.")                               
        self.ac.cancel_goal()

    def odomCallback(self, msg):
        self.current_position = [msg.pose.pose.position.x,msg.pose.pose.position.y,0]
        #print(math.dist(self.current_position,self.current_destination))
        if math.dist(self.current_position,self.current_destination) < 0.1:
            self.robot_state *= -1
            self.current_destination = self.destinationDict[self.robot_state]
            time.sleep(1)
            if self.robot_state != "picking":
                self.send_goal()
        # print(self.odom)

    def send_stop(self):
        self.stop_msg.linear.x = 0
        self.stop_msg.linear.y = 0
        self.stop_msg.linear.z = 0
        self.stop_msg.angular.x = 0
        self.stop_msg.angular.y = 0
        self.stop_msg.angular.z = 0
        self.stop_vel_pub.publish(self.stop_msg)

    def send_goal(self):
        if self.isFirstMove:
            current_destination = self.firstDestination
            self.goal.target_pose.header.frame_id = 't265_odom_frame'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = current_destination[0]
            self.goal.target_pose.pose.position.y = current_destination[1]
            q = tf.transformations.quaternion_from_euler(0, 0, self.current_destination[2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            self.ac.send_goal(self.goal)
            succeeded = self.ac.wait_for_result(rospy.Duration(1))
            state = self.ac.get_state()

            print(state)
            if succeeded:
                rospy.loginfo("Arrived destination")
            else:
                rospy.loginfo("Not arrvied destination")
            rospy.sleep(0.1)            
            self.isFirstMove = 0
            
            current_destination = self.destinationDict[1]
            self.goal.target_pose.header.frame_id = 't265_odom_frame'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = current_destination[0]
            self.goal.target_pose.pose.position.y = current_destination[1]
            q = tf.transformations.quaternion_from_euler(0, 0, self.current_destination[2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            self.ac.send_goal(self.goal)
            succeeded = self.ac.wait_for_result(rospy.Duration(1))
            state = self.ac.get_state()

            print(state)
            if succeeded:
                rospy.loginfo("Arrived destination")
            else:
                rospy.loginfo("Not arrvied destination")
            self.ac.send_goal(self.goal)
        else:
            self.goal.target_pose.header.frame_id = 't265_odom_frame'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = self.current_destination[0]
            self.goal.target_pose.pose.position.y = self.current_destination[1]
            q = tf.transformations.quaternion_from_euler(0, 0, self.current_destination[2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            # if(self.x_pos < self.destination[0]):
            #     self.goal.target_pose.pose.position.x = self.destination[0]
            #     self.goal.target_pose.pose.position.y = self.destination[1]
            #     q = tf.transformations.quaternion_from_euler(0, 0, self.destination[2])
            #     self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            # if(self.destination[0] <= self.x_pos):
            #     self.goal.target_pose.pose.position.x = self.home_position[0]
            #     self.goal.target_pose.pose.position.y = self.home_position[1]
            #     q = tf.transformations.quaternion_from_euler(0, 0, self.home_position[2])
            #     self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            
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
        client,address = self.server.accept()
        while not rospy.is_shutdown():
            
            print("------------------------------------")
            print("[*] Connected!! [ Source : {}]".format(address))
            try:
                data = client.recv(self.buffer_size)
            except:
                print("recive time out")
                pass
            print("[*] Received Data : {}".format(data))

            if(data == b"Move"):
                print("Sending Goal")
                self.xarm_state = "waiting"
                self.send_goal()
                client.sendall(b"Searching")
            if(data == b"Stop"):
                print("Cancel destination and Sending Stop")
                self.xarm_state = "picking"
                self.ac.cancel_goal()
                self.send_stop()
                client.sendall(b"StartPick")
            try:
                client.send(b"Received by dolly")
            except:
                pass
        client.close()


if __name__ == '__main__':
    try:
        rospy.init_node('TCP_server', anonymous=True)
        tcp_server = TCP_Server()
        tcp_server.process()
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TCP_server is finished.")

        