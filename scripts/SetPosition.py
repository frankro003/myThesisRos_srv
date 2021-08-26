#!/usr/bin/env python
import rospy
from digitalTwins_msgs.srv import(SetJointState,SetJointStateResponse)
#############################
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
######################################################

class SetpositionMonitor(object):
    def __init__(self):
        self.RobotTest = _MoveGroupRobot()
        pass
    def set_position(self,req):
        rospy.loginfo('SetPosition Called')
        rospy.loginfo("%s"%(req.joint[0]))
        response = SetJointStateResponse()
        response.ok = True
        self.RobotTest._moveRobot(req.joint)
        return response


class _MoveGroupRobot:
    def __init__(self):
        rospy.loginfo('moveGroup init')
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def _moveRobot(self,_joint):
        self._joint = _joint
        numPoint = len(self._joint)/6
        k = 0
        for i in range(numPoint):
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = self._joint[0+k]
            joint_goal[1] = self._joint[1+k]
            joint_goal[2] = self._joint[2+k]
            joint_goal[3] = self._joint[3+k]
            joint_goal[4] = self._joint[4+k]
            joint_goal[5] = self._joint[5+k]
            self.move_group.go(joint_goal, wait=True)
            k=k+6
        self.move_group.stop()


def main():    
    rospy.init_node('robot_server')
    monitor = SetpositionMonitor()
    set_pose = rospy.Service('set_position',SetJointState,monitor.set_position)
    rospy.spin()

if __name__ == '__main__':
    main()