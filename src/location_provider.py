#!/usr/bin/env python

__author__ = 'daniel'

import rospy
from location_provider.srv import *

class location:
    name = ""
    center = [0, 0]
    points = []

    def __init__(self, _name, point1, point2): # going to keep this as p1 and p2 to force 2 points.
        name = _name
        self.center[0] = (point1[0] + point2[0])/2
        self.center[1] = (point1[1] + point2[1])/2
        self.points.append(point1) # storing this in a list to open up the possibility of something better than a AABB
        self.points.append(point2)

    def contains_point(self, point): # sad formula is unoptimized, move on.
        if (abs(point[0]-points[0][0])+abs(point[0]-points[1][0]) <= abs(points[0][0]-points[1][0])) and \
                (abs(point[1]-points[0][1])+abs(point[1]-points[1][1]) <= abs(points[0][1]-points[1][1])):
            return True

locations = []
locations.append(location('kitchen', (0, 1), (2, 2)))

def get_location_from_pose_function(rec):
    y = rec.robotPose.position.y
    x = rec.robotPose.position.x
    for nom in locations:
        if nom.contains_point((x,y)):
            return getLocationFromPoseRequest(nom.name)
        else:
            return getLocationFromPoseRequest("nowhere")

def get_location_list_function():
    output = ""
    for nom in locations:
        output+=nom.name

    return output

if __name__ =="__main__":
    rospy.init_node("location_provider")
    a = rospy.Service('get_location_from_pose', getLocationFromPose, get_location_from_pose_function)
    b = rospy.Service('get_location_list', getLocationList, get_location_list_function)
    rospy.spin()