import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Action:

    def __init__(self, agv):

        # Agv
        self.agv = agv

        # Action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.active_cb, auto_start=False)
        #self.server.start()

    def move_to_node(self, node):
        
        # Get node location
        print("AGV " + str(self.agv.id) + ":        Move to node " + node)
        loc = self.agv.graph.nodes[node].pos

        # Connect to server
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = -3.0
        goal.target_pose.pose.orientation.x = 1.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0

        # Send goal
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def cancel_goal(self):
        self.client.cancel_goal()

    def active_cb(self):
        rospy.loginfo("Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose received: " + str(feedback))

    def done_cb(self, status, result):
        if status == 2:
            rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")
        if status == 3:
            rospy.loginfo("Goal pose reached") 
            rospy.signal_shutdown("Final goal pose reached!")
            return
        if status == 4:
            rospy.loginfo("Goal pose was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose aborted, shutting down!")
            return
        if status == 5:
            rospy.loginfo("Goal pose has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose rejected, shutting down!")
            return
        if status == 8:
            rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")

    def pick(self):
        print("AGV " + str(self.agv.id) + ":        Pick")

    def place(self):
        print("AGV " + str(self.agv.id) + ":        Place")