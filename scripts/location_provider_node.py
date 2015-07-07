#!/usr/bin/env python

__author__ = 'daniel'

import rospy
from location_provider.srv import *

import rosplan_interface as interface

from geometry_msgs.msg import PoseWithCovariance, Pose, Point,\
    Quaternion
from move_base_msgs.msg import *
from actionlib import *

pose = PoseWithCovariance(Pose(Point(0, 0, 0),
                               Quaternion(0, 0, 0, 1)),
                          [0.0]*36)


# class location:
#     name = ""
#     center = [0, 0]
#     points = []
#
#     def __init__(self, _name, point1, point2):
#         # going to keep this as p1 and p2 to force 2 points.
#         name = _name
#         self.center[0] = (point1[0] + point2[0])/2
#         self.center[1] = (point1[1] + point2[1])/2
#         self.points.append(point1)
#         # storing this in a list to open up the
#         # possibility of something better than a AABB
#         self.points.append(point2)
#
#     def as_pose(self):
#         return Pose(Point(self.center[0], self.center[1], 0),
#                     Quaternion(0,0,0,1))
#
#     def contains_point(self, point): # sad formula is unoptimized, move on.
#         if (abs(point[0]-self.points[0][0])+abs(point[0]-self.points[1][0]) <= abs(self.points[0][0]-self.points[1][0])) and \
#                 (abs(point[1]-self.points[0][1])+abs(point[1]-self.points[1][1]) <= abs(self.points[0][1]-self.points[1][1])):
#             return True
#
# locations = []
# locations.append(location('kitchen', (0, 1), (2, 2)))

# def get_location_from_pose_function(rec):
#     y = rec.robotPose.position.y
#     x = rec.robotPose.position.x
#     for nom in locations:
#         if nom.contains_point((x, y)):
#             return getLocationFromPoseRequest(nom.name)
#         else:
#             return getLocationFromPoseRequest("nowhere")
#
# def get_location_list_function():
#     output = ""
#     for nom in locations:
#         output+=nom.name
#
#     return output


def roam():
    for location in interface.list_instances('location'):
        interface.add_goal('hasvisited', x=location)
    interface.add_instance('location', 'starting_point')
    interface.add_predicate('robotat', x='starting_point')
    interface.plan()


def set_location_function(name, pose_in=None):
    global pose
    if pose_in is None:
        pose_in = pose.pose
    # print pose
    # rospy.INFO('Setting location to %s' % name)
    interface.add_instance('location', name, pose_in)
    # interface.add_instance('location', name, pose)


def update_pose_function(arg):
    global pose
    pose = arg


class moveAction(interface.Action):
    name = "move"
    def __init__(self, *args, **kwargs):
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(secs=10))
        super(moveAction, self).__init__(*args, **kwargs)

    def start(self, loca, locb):
        goal = MoveBaseActionGoal()
        goal.goal.target_pose = interface.get_instance(locb, Pose._type)
        print goal.goal.target_pose
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_result() != GoalStatus.SUCCEEDED:
            raise Exception("Moving failed!")

    def cancel(self):
        client.cancel_goal()


locations = {"tony_office": Pose(Point(2.495, 1.964, 0.000), Quaternion(0.000, 0.000, -0.044, 0.999)),
             "room_five_sixty": Pose(Point(2.581, 4.985, 0.000), Quaternion(0.000, 0.000, -0.028, 1.000)),
             "kitchen": Pose(Point(-8.588, 28.927, 0.000), Quaternion(0.000, 0.000, 1.000, -0.015)),
             "cubicle": Pose(Point(-0.717, 0.897, 0.000), Quaternion(0.000, 0.000, 0.734, 0.680)),
             "conference_room": Pose(Point(1.056, 35.795, 0.000), Quaternion(0.000, 0.000, 0.048, 0.999)),
             "it": Pose(Point(-15.275, 36.182, 0.000), Quaternion(0.000, 0.000, 0.741, 0.672)),
             "entrance": Pose(Point(-4.926, 5.199, 0.000), Quaternion(0.000, 0.000, 0.977, -0.211))}

if __name__ == "__main__":
    rospy.init_node("location_provider")
    interface.init_rosplan()
#     a = rospy.Service(OB'get_location_from_pose',
#                       getLocationFromPose,
#                       get_location_from_pose_function)
#     b = rospy.Service('get_location_list',
#                       getLocationList,
#                       get_location_list_function)
    c = rospy.Service('setLocation',
                      setLocation,
                      set_location_function)
    amcl_pose = rospy.Subscriber('amcl_pose',
                                 PoseWithCovariance,
                                 update_pose_function, 1, 1)

    for i in locations:
        set_location_function(i, locations[i])
    roam()
    rospy.spin()
