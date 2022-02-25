import os
import yaml
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

node_locations = {}
global counter
counter = 1

def amcl_callback(msg):

	# Global variable
	global counter

	# Get position
	x_loc = msg.pose.pose.position.x
	y_loc = msg.pose.pose.position.y

	# Get orientation
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(_, _, yaw) = euler_from_quaternion (orientation_list)
	theta = yaw
	print("Received pose: x=" + str(x_loc) + ' y=' + str(y_loc) + ' theta=' + str(theta))

	# Node name
	node_name = 'pos_' + str(counter)
	counter += 1

	# Save node
	node_locations[node_name] = [x_loc, y_loc, theta]

def shutdown():

	this_dir = os.path.dirname(os.path.dirname(__file__))
	data_path = os.path.join(this_dir, "locations", "test.yaml")
	with open(data_path, 'w') as file:
		yaml.dump(node_locations, file)


if __name__ == "__main__":

	# Init node
	rospy.init_node('save_positions')

	# Init subscriber
	status_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, amcl_callback)

	# Shutdown routine
	rospy.on_shutdown(shutdown)

	# Spin
	rospy.spin()
