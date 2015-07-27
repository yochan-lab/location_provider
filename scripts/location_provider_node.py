#!/usr/bin/env python

__author__ = 'daniel'
import traceback
import rospy
from location_provider.srv import *

import rosplan_interface as interface

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point,\
    Quaternion
from move_base_msgs.msg import *
from actionlib import *
from threading import Thread

pose = PoseWithCovarianceStamped(Pose(Point(0, 0, 0),
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


#def roam():
#    for location in interface.list_instances('location'):
#        interface.add_goal('hasvisited', x=location)
#
#    interface.plan()


def set_location_function(name, pose_in=None):
    global pose
    if pose_in is None:
        pose_in = pose.pose
    # print pose
    # rospy.INFO('Setting location to %s' % name)
    interface.add_instance('location', name, pose_in)
    # interface.add_instance('location', name, pose)


def update_pose_function(arg, *args, **kwargs):
    global pose
    pose = arg


class moveAction(interface.Action):
    name = "move"
    def __init__(self, *args, **kwargs):
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(secs=10))
        self.cancelled = False
        super(moveAction, self).__init__(*args, **kwargs)
        print "DONE WITH INIT"

    def start(self, loca, locb):
        self.cancelled = False
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.pose = interface.get_instance('location', locb, Pose._type)[0]
        goal.goal.target_pose.header.frame_id = "map"
        print goal.goal.target_pose.pose
        self.client.send_goal(goal.goal)
        Thread(target=self.bkgd).start() 
    
    def bkgd(self):
        self.client.wait_for_result()
        if not self.cancelled:
	    self.finish()

    def finish(self):
        interface.rm_predicate('robotat', x=self.arguments['loca'])
        interface.add_predicate('robotat', x=self.arguments['locb'])
        interface.add_predicate('hasvisited', x=self.arguments['locb'])
        self.report_success()
#         if self.client.get_result() != GoalStatus.SUCCEEDED:
#             print self.client.get_result()
#             raise Exception("Moving failed!")

    def cancel(self):
        print "cancelling goal"
        self.cancelled = True
	self.client.cancel_goal()
	print "goal canceled"


locations = {"home": Pose(Point(-1.395, 1.091, 0.000), Quaternion(0.000, 0.000, 0.701, 0.713)),
	"office": Pose(Point(2.237, 2.296, 0.000), Quaternion(0.000, 0.000, -0.038, 0.999)),
	"rao": Pose(Point(2.281, 5.228, 0.000), Quaternion(0.000, 0.000, 0.006, 1.000)),
	"monica": Pose(Point(-1.813, -2.504, 0.000), Quaternion(0.000, 0.000, -0.347, 0.938)),
	"entrance": Pose(Point(-5.286, 5.447, 0.000), Quaternion(0.000, 0.000, 0.992, -0.127)),
	"bathroom": Pose(Point(-4.848, 17.113, 0.000), Quaternion(0.000, 0.000, 1.000, -0.002)),
	"fred": Pose(Point(-0.586, 19.598, 0.000), Quaternion(0.000, 0.000, -0.725, 0.689)),
	"kitchen": Pose(Point(-14.417, 28.528, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)),
	"classroom": Pose(Point(-23.638, 19.286, 0.000), Quaternion(0.000, 0.000, 0.992, 0.128)),
	"brint": Pose(Point(-22.049, 39.687, 0.000), Quaternion(0.000, 0.000, 0.721, 0.693)),
	"IT": Pose(Point(-21.433, 37.345, 0.000), Quaternion(0.000, 0.000, -0.034, 0.999)),
	"conferenceroom": Pose(Point(3.326, 38.098, 0.000), Quaternion(0.000, 0.000, 0.723, 0.691)),
	"chattin": Pose(Point(-26.513, -7.674, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))}

def handle_location_list(req):
	return GetLocationListResponse(locations.keys())

if __name__ == "__main__":
    rospy.init_node("location_provider")
    # interface.init_rosplan()
#     a = rospy.Service(OB'get_location_from_pose',
#                       getLocationFromPose,
#                       get_location_from_pose_function)
    b = rospy.Service('get_location_list',
                       GetLocationList,
                       handle_location_list)
#    c = rospy.Service('setLocation',
#                      setLocation,
#                      set_location_function)
    amcl_pose = rospy.Subscriber('amcl_pose',
                                 PoseWithCovarianceStamped,
                                 update_pose_function, 1, 1)
    interface.init_rosplan()
    #for i in locations:
    #    set_location_function(i, locations[i])
#
 #   interface.add_instance('location', 'starting_point')
  #  interface.add_predicate('robotat', x='starting_point')
    
    #roam()
    rospy.spin()
