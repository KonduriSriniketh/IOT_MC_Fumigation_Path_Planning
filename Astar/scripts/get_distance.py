#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import math


def path_callback(path):
    #rospy.loginfo(rospy.get_caller_id() + "Path waypoint number = " + str(len(path.poses)))
    print("len"+str(len(path.poses)))
    total_dis = calculate_path_distance(path)
    print("total distance" + str(total_dis))

       
def calculate_path_distance(path_msg):
    total_distance = 0.0
    for i in range(len(path_msg.poses) - 1):
        total_distance += calculate_distance(path_msg.poses[i], path_msg.poses[i + 1])
    return total_distance

def calculate_distance(point1, point2):
    # Euclidean distance between two points in 2D space
    return math.sqrt((point2.pose.position.x - point1.pose.position.x)**2 + (point2.pose.position.y - point1.pose.position.y)**2)
    


def listener():
    rospy.init_node('distance_calculater', anonymous=True)
    rospy.Subscriber("nav_path", Path, path_callback)

    rospy.spin()

if __name__ == "__main__":
    listener()
