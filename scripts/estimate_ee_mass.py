#!/usr/bin/env python

import sys

# ROS
import rospy
import sensor_msgs.msg

# ROS custom
import robot_msgs.srv
import robot_control.control_utils as controlUtils

# other stuff
import yaml
import time


def moveToJointPosition(jointPosition):
	maxJointDegreesPerSecond = 30

	jointState = sensor_msgs.msg.JointState()
	jointState.header.stamp = rospy.Time.now()

	jointState.position = jointPosition
	jointState.name = controlUtils.getIiwaJointNames()

	numJoints = len(jointState.name)
	assert len(jointPosition) == numJoints
	jointState.velocity = [0]*numJoints
	jointState.effort = [0]*numJoints

	rospy.wait_for_service('robot_control/MoveToJointPosition')
	s = rospy.ServiceProxy('robot_control/MoveToJointPosition', robot_msgs.srv.MoveToJointPosition)
	response = s(jointState, maxJointDegreesPerSecond)

	print "response ", response


if __name__ == '__main__':
	pose_file = open('stored_poses.yaml', 'r')
	data = pose_file.read()
	dict_poses = yaml.load(data)

	rospy.init_node('MoveRobotToStoredPoses')
	for pose in dict_poses:
		jointPosition = dict_poses[pose]['joint_angles']
		print 'Moving to:', jointPosition 
		moveToJointPosition(jointPosition)
		time.sleep(2)

	while not rospy.is_shutdown():
		rospy.spin()

	dict_haha = {'haha': [0.0, 0.1, 0.2], 'hehe':[-1, -2, -3], 'hahahahaha':[-2, -3, -4]}
	stream = file('document.yaml', 'w')
	yaml.dump(dict_haha, stream)