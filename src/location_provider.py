#!/usr/bin/env python

__author__ = 'daniel'

import rospy
from location_provider.srv import *

class location:
    name = ""
    center = (0,0)
    def __init__(self, _name, point1, point2, point3):
        name = _name



def get_location_from_pose_function(rec):
    y = rec.robotPose.position.y
    x = rec.robotPose.position.x

def get_location_list_function():
   return

if __name__ =="__main__":
    rospy.init_node("location_provider")
    a = rospy.Service('get_location_from_pose', getLocationFromPose, get_location_from_pose_function)
    b = rospy.Service('get_location_list', getLocationList, get_location_list_function)
    rospy.spin()