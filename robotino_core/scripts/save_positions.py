import os
import yaml
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

node_locations = {'node_locations': []}

def amcl_callback(msg):

	# Get position
	x_loc = msg.pose.pose.position.x
	y_loc = msg.pose.pose.position.y

	# Get orientation
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(_, _, yaw) = euler_from_quaternion (orientation_list)
	theta = yaw
	print("Received pose: x=" + str(x_loc) + ' y=' + str(y_loc) + ' theta=' + str(theta))

	node_locations['node_locations'].append(tuple([x_loc, y_loc, theta]))

	print(node_locations)

def shutdown():

	this_dir = os.path.dirname(os.path.dirname(__file__))
	data_path = os.path.join(this_dir, "src/robotino_core/params", "test.yaml")
	with open(data_path, 'w') as file:
		documents = yaml.dump(node_locations, file)


if __name__ == "__main__":

	# Init node
	rospy.init_node('save_positions')

	# Init subscriber
	status_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, amcl_callback)

	# Shutdown routine
	rospy.on_shutdown(shutdown)

	# Spin
	rospy.spin()
