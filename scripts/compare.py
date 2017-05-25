#!/usr/bin/env python
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

import rosbag
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal


def resolve_id(bag):
	for topic, msg, t in bag.read_messages(topics=['/arm_controller/follow_joint_trajectory/status', '/follow_joint_trajectory/status']):
		for status in msg.status_list:
			if status.status == 1:
				return status.goal_id.id

def resolve_start_time(bag):

	id = resolve_id(bag)

	for topic, msg, t in bag.read_messages(topics=['/arm_controller/follow_joint_trajectory/status', '/follow_joint_trajectory/status']):
		for status in msg.status_list:
			if status.status == 1 and status.goal_id.id == id:
				return msg.header.stamp

def resolve_end_time(bag):

	id = resolve_id(bag)

	for topic, msg, t in bag.read_messages(topics=['/arm_controller/follow_joint_trajectory/status', '/follow_joint_trajectory/status']):
		for status in msg.status_list:
			if status.status == 3 and status.goal_id.id == id:
				return msg.header.stamp


jorder =  { 'shoulder_pan_joint': 1, 
         	'shoulder_lift_joint' : 2, 
			'elbow_joint': 3, 
			'wrist_1_joint' : 4, 
			'wrist_2_joint' : 5, 
			'wrist_3_joint' : 6 }

def plot_js(bag, start, end, plots = [], style = '-'):

	data = []

	for topic, msg, t in bag.read_messages(topics=['/joint_states']):
		if msg.header.stamp >= start and msg.header.stamp < end:
			time_stamp = (msg.header.stamp - start).to_sec()
				
			row = np.append(time_stamp, msg.position)
			data.append(row)

			# print(row)

	# for i in range(len(msg.name)):
	# 	jdict[msg.name[i]] = i

	# print(jdict)

	data = np.array(data)

	for i in range(6):



		index = jorder[msg.name[i]]

		plt.subplot(3, 2, index)
		
		plt.plot(data[:, 0], data[:, i+1], ls = style, hold=True)
		plt.ylabel(msg.name[i])
		plt.xlabel('time')

	print(data[:,0])


def main(args):
	# print(len(args))
	if len(args) < 3:
		print("Error: Too few arguments")

	bag_a = rosbag.Bag(args[1])
	bag_b = rosbag.Bag(args[2])

	id_a = resolve_id(bag_a)
	start_a = resolve_start_time(bag_a)
	end_a = resolve_end_time(bag_a)



	id_b = resolve_id(bag_b)
	start_b = resolve_start_time(bag_b)
	end_b = resolve_end_time(bag_b)

	offset = rospy.Duration(0.17)

	plot_js(bag_a, start_a, end_a)	
	plot_js(bag_b, start_b + offset, end_b + offset, style='-.')

	plt.show()
	# print(id)
	# print(start)
	# print(end)
	# print((end-start).secs)

	# for topic, msg, t in bag.read_messages(topics=['/follow_joi', 'numbers']):
	#     print msg
	bag.close()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
