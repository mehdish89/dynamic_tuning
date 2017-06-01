#!/usr/bin/env python
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

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
			return msg.header.stamp
			if status.status == 1 and status.goal_id.id == id:
				return msg.header.stamp

def resolve_end_time(bag):

	id = resolve_id(bag)

	for topic, msg, t in bag.read_messages(topics=['/arm_controller/follow_joint_trajectory/status', '/follow_joint_trajectory/status']):
		for status in msg.status_list:
			tt = msg.header.stamp
			if status.status == 3 and status.goal_id.id == id:
				return msg.header.stamp

	return tt


jorder =  { 'shoulder_pan_joint': 1, 
         	'shoulder_lift_joint' : 2, 
			'elbow_joint': 3, 
			'wrist_1_joint' : 4, 
			'wrist_2_joint' : 5, 
			'wrist_3_joint' : 6 }


def plot_data(data, plots = [], style = '-'):
	data = np.array(data)

	for i in range(6):
		index = i + 1 #jorder[msg.name[i]]

		plt.subplot(3, 2, index)
		
		plt.plot(data[:, 0], data[:, i+1], ls = style, hold=True)
		# plt.ylabel(msg.name[i])
		plt.xlabel('time')



	print(data[:,0])

def plot_js(bag, start, end, plots = [], style = '-'):

	data = []

	for topic, msg, t in bag.read_messages(topics=['/joint_states']):
		if msg.header.stamp >= start and msg.header.stamp < end:
			time_stamp = (msg.header.stamp - start).to_sec()
				
			row = np.append(time_stamp, msg.position)
			data.append(row)

	data = np.array(data)

	for i in range(6):
		index = jorder[msg.name[i]]

		plt.subplot(3, 2, index)
		
		plt.plot(data[:, 0], data[:, i+1], ls = style, hold=True)
		plt.ylabel(msg.name[i])
		plt.xlabel('time')

	# print(data[:,0])

			# print(row)

	# for i in range(len(msg.name)):
	# 	jdict[msg.name[i]] = i

	# print(jdict)

	


def extract_data(bag, start, end):

	data = []

	for topic, msg, t in bag.read_messages(topics=['/joint_states']):
		if msg.header.stamp >= start and msg.header.stamp <= end:
			time_stamp = (msg.header.stamp - start).to_sec()
				
			row = np.append(time_stamp, msg.position)
			data.append(row)

	return np.array(data)

def find_error(t_a, f_a, t_b, f_b, offset = 0., resolution = rospy.Duration(0.01)):

	dt = resolution.to_sec()

	# t_a += offset

	min_t = max(np.min(t_a + offset), np.min(t_b))
	max_t = min(np.max(t_a + offset), np.max(t_b))	
	t_new = np.arange(min_t, max_t, dt)

	# print(t_a)
	# print(min_t, max_t)
	# print(t_new - offset)
	# print(t_new)

	y_new_a = f_a(t_new - offset)
	y_new_b = f_b(t_new)

	# data_a = np.concatenate((t_new.reshape(len(t_new),1), y_new_a), axis=1)
	# data_b = np.concatenate((t_new.reshape(len(t_new),1), y_new_b), axis=1)


	# plot_data(data_a)
	# plot_data(data_b)
	# plt.draw()



	error = (sum(sum(abs(y_new_a-y_new_b)**2))) / len(t_new)
	# print(error)
	# print("###########")

	return error



def find_offset(bag_a, start_a, end_a, bag_b, start_b, end_b, resolution = rospy.Duration(0.01), off_range = rospy.Duration(2.)):
	
	data_a = extract_data(bag_a, start_a, end_a)
	data_b = extract_data(bag_b, start_b, end_b)

	t_a = data_a[:,0]
	y_a = data_a[:,1:]

	t_b = data_b[:,0]
	y_b = data_b[:,1:]

	f_a = interp1d(t_a, y_a, axis = 0, fill_value = "extrapolate")
	f_b = interp1d(t_b, y_b, axis = 0, fill_value = "extrapolate")

	print(t_a)
	print(t_b)
	# return

	offsets = np.arange(-off_range.to_sec(), off_range.to_sec(), resolution.to_sec())

	min_error = 1000000000000.
	best_offset = 0.

	for offset in offsets:
		error = find_error( t_a, 
							f_a, 
							t_b, 
							f_b, 
							offset,
							resolution)

		if error < min_error:
			min_error = error
			best_offset = offset

		# print("offset", offset)
	# if offset > 0.:
	# 	plt.show()

	return rospy.Duration(best_offset)




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

	# offset = rospy.Duration(0.17)
	offset = rospy.Duration(0.60)

	# plt.ion()

	offset = find_offset(bag_a, start_a, end_a, bag_b, start_b, end_b)
	print(offset)

	# print(start_b, end_b)

	plot_js(bag_a, start_a, end_a)	
	plot_js(bag_b, start_b + offset, end_b + offset)#, style='-.')

	# find_error(bag_a, start_a, end_a, bag_b, start_b, end_b)

	mng = plt.get_current_fig_manager()
	mng.full_screen_toggle()

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
