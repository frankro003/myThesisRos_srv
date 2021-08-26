#!/usr/bin/env python

import rospy
from digitalTwins_msgs.srv import (GetClosest,GetClosestResponse,
                                    GetDistance,GetDistanceResponse)

class LandmarkMonitor(object):
    def __init__(self):
        pass
    def get_closest(self,req):
        rospy.loginfo('GetClosest called')
        response = GetClosestResponse()
        response.name = 'Universal Robot'
        return response
    def get_distance(self,req):
        rospy.loginfo('GetDistance called with{}'.format(req.name))
        response = GetDistanceResponse()
        return response


def main():
    rospy.init_node('landmark_server')
    monitor = LandmarkMonitor()
    get_closest = rospy.Service('get_closest',GetClosest,monitor.get_closest)
    get_distance = rospy.Service('get_distance',GetDistance,monitor.get_distance)
    rospy.spin()

if __name__ == '__main__':
    main()