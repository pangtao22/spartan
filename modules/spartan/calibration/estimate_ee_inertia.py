#!/usr/bin/env python

import os
import sys

# ROS
import rospy
import sensor_msgs

# ROS custom
import robot_msgs.srv
import robot_control.control_utils as controlUtils

# other stuff
import time

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils



class EstimateEndEffectorInertia():
	def __init__(self):
		self.robot_service = rosUtils.RobotService.makeKukaRobotService()
		self.joint_state_subscriber = \
			rosUtils.SimpleSubscriber("/joint_states", sensor_msgs.msg.JointState)
		self.spartan_source_dir = spartanUtils.getSpartanSourceDir()


	def CollectData(self):
		"""
		Moves the robot to poses in stored_poses.yaml and record the joint torques at those pose. 
		"""
		self.LoadStoredPoses()
		self.joint_state_subscriber.start()

		for pose in self.dict_poses:
			q = self.dict_poses[pose]['joint_angles']
			print 'Moving to:', q 
			self.robot_service.moveToJointPosition(q)
			time.sleep(2)

			joint_state_msg = self.joint_state_subscriber.waitForNextMessage()
			self.dict_poses[pose]['torque_external'] = list(joint_state_msg.effort)
			print "Toqrue_external: ", joint_state_msg.effort

		self.joint_state_subscriber.stop()
		self.SaveDataToFile()



	def LoadStoredPoses(self):
		stored_poses_file = os.path.join(self.spartan_source_dir, 'src', 'catkin_projects',
			'station_config', 'RLG_iiwa_1', 'ee_inertia_calibration', 'stored_poses.yaml')

		self.dict_poses = spartanUtils.getDictFromYamlFilename(stored_poses_file)


	def SaveDataToFile(self):
		save_dir = os.path.join(self.spartan_source_dir, 'sandbox', 'ee_inertia_calibraton')
		filename = os.path.join(save_dir, 'ee_inertia_calibraton_data.yaml')

		if not os.path.isdir(save_dir):
			os.mkdir(save_dir)

		spartanUtils.saveToYaml(self.dict_poses, filename)

	def Run(self):
		"""
		Runs the calibration procedure 
		"""
		self.CollectData()

if __name__ == "__main__":
	rospy.init_node('EstimateEndEffectorInertia')
	estimate_ee_inertia = EstimateEndEffectorInertia()
	estimate_ee_inertia.Run()


