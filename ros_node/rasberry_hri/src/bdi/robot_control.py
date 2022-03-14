import rospy
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from topological_navigation.route_search import TopologicalRouteSearch
from strands_navigation_msgs.msg import TopologicalMap

class RobotControl:
    def __init__(self, robot_id="thorvald_014"):
        rospy.loginfo("RCO: Initializing Robot Control")
        self.robot_id = robot_id
        rospy.loginfo("RCO: Waiting for Topological map ...")
        try:
            msg = rospy.wait_for_message('/topological_map', TopologicalMap, timeout=10.0)
            self.top_map = msg
            self.lnodes = msg.nodes
        except rospy.ROSException :
            rospy.logwarn("Failed to get topological map")
            return
        rospy.loginfo("RCO: Waiting for Robot Control Server...")
        self.robot_control = actionlib.SimpleActionClient(
            "/{:}/topological_navigation".format(self.robot_id), GotoNodeAction
        )
        self.robot_control.wait_for_server()
        rospy.loginfo("RCO: Found Robot Control Server")

    def move_to(self, goal):
        rospy.loginfo("RCO: Requesting Navigation to: {:}".format(goal))
        navgoal = GotoNodeGoal()
        navgoal.target = goal

        # Sends the goal to the action server.
        self.robot_control.send_goal(navgoal)

    # Waits for the server to finish performing the action.
    def get_result(self):
        return self.robot_control.get_result()  # A FibonacciResult

    def cancel_movement(self):
        self.robot_control.cancel_all_goals()

    def get_path_length(self, start, end):
        rsearch = TopologicalRouteSearch(self.top_map)
        route = rsearch.search_route(start, end)
        return len(route.edge_id)
