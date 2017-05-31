#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import *

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

def get_services():
	rospy.wait_for_service('/gazebo/advertise_joint_params')

	try:
		advertise_params = rospy.ServiceProxy('/gazebo/advertise_joint_params', ParamListRequest)
		res = advertise_params()
		return res.params
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def set_param(service_name, value):
    rospy.wait_for_service("gazebo/" + service_name)
    try:
        set_param_ = rospy.ServiceProxy("gazebo/" + service_name, SetParamFloat)
        res = set_param_(value)
        return res.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_params(value):
	services = get_services()
	for service_name in services:
		set_param(service_name, value)

def dyn_rec_callback(config, level):
	# print(config)
	# print(level)

	for key in config:
		if key == "groups":
			continue

		service_name = key.replace("___", "/")
		set_param(service_name, config[key])

	# rospy.loginfo("Received reconf call: " + str(config))
	return config

if __name__ == '__main__':
    rospy.init_node('ddynrec')

    # Create a D(ynamic)DynamicReconfigure
    ddynrec = DDynamicReconfigure("")

    services = get_services()

    for service_name in services:
		print(service_name)
		ddynrec.add_variable(service_name.replace("/", "___"), "float/double variable", 0.0, 0.0, 1000.0)	

    # Add variables (name, description, default value, min, max, edit_method)
    # ddynrec.add_variable("decimal", "float/double variable", 0.0, -1.0, 1.0)
    # ddynrec.add_variable("integer", "integer variable", 0., -1., 1.)
    # ddynrec.add_variable("bool", "bool variable", True)
    # ddynrec.add_variable("string", "string variable", "string dynamic variable")
    # enum_method = ddynrec.enum([ ddynrec.const("Small",      "int", 0, "A small constant"),
    #                    ddynrec.const("Medium",     "int", 1, "A medium constant"),
    #                    ddynrec.const("Large",      "int", 2, "A large constant"),
    #                    ddynrec.const("ExtraLarge", "int", 3, "An extra large constant")],
    #                  "An enum example")
    # ddynrec.add_variable("enumerate", "enumerate variable", 1, 0, 3, edit_method=enum_method)

    # Start the server
    ddynrec.start(dyn_rec_callback)

    rospy.spin()