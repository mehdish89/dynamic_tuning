#!/usr/bin/env python
import math
import numpy as np
import random
import string

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

def js_callback(msg):
    global last_js
    last_js = msg


def gen_traj(joint_limits = np.array([[-math.pi, math.pi]] * 6), 
             vel_limit = 0.1,
             acc_mod = 0.1,
             time_step = 0.1,
             action_step = 10,
             duration = 10):

    traj = JointTrajectory()
    traj.joint_names = [ 'shoulder_pan_joint', 
                         'shoulder_lift_joint', 
                         'elbow_joint', 
                         'wrist_1_joint', 
                         'wrist_2_joint', 
                         'wrist_3_joint' ]

    traj.header.frame_id = '/world'

    t = 0.
    iters = 0

    init_pose = np.array(last_js.position) ##### TODO: remove the dependency on global pose ####
    init_pose[0:3] = init_pose[2::-1]

    pos = np.array([0.] * 6)
    vel = np.array([0.] * 6)
    acc = np.array([0.] * 6)

    while t < duration:

        # print("############################")
        # print(t)
        # print(pos)
        # print(vel)
        # print(acc)

        point = JointTrajectoryPoint()
        point.positions = pos + init_pose
        point.velocities = [0.] * 6
        point.accelerations = [0.] * 6
        point.time_from_start = rospy.Duration(t)

        traj.points.append(point)
        
        pos += vel * time_step
        pos = np.minimum(pos, joint_limits[:,1])
        pos = np.maximum(pos, joint_limits[:,0])

        vel += acc * time_step
        vel = np.minimum(vel, vel_limit)
        vel = np.maximum(vel, -vel_limit)

        # print (iters % action_step)


        if iters % action_step == 0:
            acc = np.random.rand(6) - 0.5
            acc = acc / np.linalg.norm(acc)
            acc *= acc_mod


        t += time_step 
        iters += 1

    point = JointTrajectoryPoint()
    point.positions = init_pose
    point.velocities = [0.] * 6
    point.accelerations = [0.] * 6
    point.time_from_start = rospy.Duration(duration + 3)

    traj.points.append(point)
    
    return traj



def main():

    # pub = rospy.Publisher('/if_any', String, queue_size=10)
    traj_pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    rospy.init_node('generate_trajectory', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, js_callback)
    
    rospy.sleep(0.5)
    while 'last_js' not in globals() and not rospy.is_shutdown(): pass

    joint_limits = np.array([[-math.pi, math.pi]] * 6)
    # joint_limits[0:2, :] = 0.
    vel_limit = 1.0
    acc = 10.0
    time_step = 0.1

    traj = gen_traj(joint_limits = joint_limits, action_step = 10, duration = 10, acc_mod = 10., vel_limit = 0.5, time_step = 0.01)

    goal = FollowJointTrajectoryActionGoal()
    goal.goal.trajectory = traj
    # goal.header.stamp = rospy.Time.now()
    # goal.goal_id.stamp = rospy.Time.now()
    # goal.goal_id.id = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(10))

    # print(goal)

    import rosbag

    bag = rosbag.Bag('test.bag', 'w')

    try:
        bag.write('/follow_joint_trajectory/goal', goal)
    finally:
        bag.close()

    traj_pub.publish(goal)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass