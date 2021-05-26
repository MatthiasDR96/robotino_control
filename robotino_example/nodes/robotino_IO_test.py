#!/usr/bin/env python

import rospy
from robotino_msgs.msg import DigitalReadings

def digitalInputsCallback(msg):

	values = msg.values
	emergency_stop = values[0] #False if emergency
	unknown2 = values[1]
	conveyor_sensor = values[2] #True if sensing object
	unknown4 = values[3]
	reset_button = values[4] #True if pressed
	unknown5 = values[5]
	unknown6 = values[6]
	unknown7 = values[7]
	print("Inputs: " + str(values))

if __name__ == "__main__":

    # Init node
	rospy.init_node("robotino_io_tetst_node")

    # Init publishers and subscribers
	digital_pub = rospy.Publisher("/set_digital_values", DigitalReadings, queue_size=10)
	digital_inputs_sub = rospy.Subscriber("/digital_readings", DigitalReadings, digitalInputsCallback)

    # Create message
	msg = DigitalReadings()
	msg.values = [False, False, False, False, False, False, False, True]
	red_light = msg.values[2]
	orange_light = msg.values[3]
	green_light = msg.values[4]
	
    # Publish
	while not rospy.is_shutdown():
		digital_pub.publish(msg)
	rospy.spin()