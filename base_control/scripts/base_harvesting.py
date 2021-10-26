#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Twist
from math import pi

class Base_Harvesting():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.pub = rospy.Publisher('/base_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.stop = Twist()
        self.stop.linear.x = 0
        self.stop.angular.z = 0

    def convert_goal(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "t265_odom_frame"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pose.position.x
        goal.target_pose.pose.position.y = goal_pose.position.y
        goal.target_pose.pose.position.z = goal_pose.position.z
        goal.target_pose.pose.orientation.w = goal_pose.orientation.w
        return goal

    def approach_shelf(self):
        print("process start")
        pos_shelf = rospy.wait_for_message("base/pos_shelf", Pose) 
        movebaseGoal = self.convert_goal(pos_shelf)
        self.client.send_goal(movebaseGoal)
        wait = self.client.wait_for_result()
        if not wait:
            print("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print("Goal Reached")

    def move_sideway(self):
        isMove = rospy.wait_for_message("base/isMove", Bool)
        if isMove:
            print("start move_sideway")
            speed = 0.2 # [m/s]
            t = Twist()
            t.linear.x = speed
            t.angular.z = 0            
            self.pub.publish(t)

        else:
            self.pub.publish(self.stop)


if __name__ == "__main__":
    try:
        rospy.init_node("Base_Harvesting", anonymous=True)
        rospy.loginfo("Starting Base_Harvesting.")
        harvest = Base_Harvesting()
        while not rospy.is_shutdown():
            harvest.approach_shelf()
            harvest.move_sideway()
        
    except rospy.ROSInterruptException:
        pass
