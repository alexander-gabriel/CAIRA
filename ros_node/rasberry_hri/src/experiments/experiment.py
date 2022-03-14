#!/usr/bin/env python
import os
import base64
import warnings
import subprocess

import paramiko
import rospy
import roslaunch
import rosparam
import rosnode

from std_srvs.srv import Empty  # , Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState  # , GetModelState
from rasberry_hri.msg import Action

from bdi.robot_control import RobotControl
from common.parameters import NS, BEHAVIOURS, CONFIG_DIRECTORY, \
    STATE_DIRECTORY, EXPERIMENT_START_DELAY


warnings.filterwarnings(action='ignore', module='.*paramiko.*')


key = paramiko.RSAKey(
    data=base64.b64decode(
        b"AAAAB3NzaC1yc2EAAAADAQABAAABAQC9/fxTMxUkCY0+27UqYXJM+RZfJjDBeCgqYb28P5iq0Kqry98Ke0Af703yyJPfaVlRG8PUkn95k+1dh1xQL8nzWeIs9oxTOwjpOliqAn2ajQ/v2J73E2wQyHhGfRWLQ+GVzO6ZU8u48mybWxOKPsk6ETG/0eQdes+zNvI7Q0K2ZryTv/BzfWzGite2I3YTDHMGjg94BdWsQJ3t+leV29nsn6eH4vYliFJcDJRexck/FvA2j3n3eqh/ahP2UTFChna3BS3EGfGTaFUaqxJyxijXtXAkZd6OGweD1D6VREX50/JwjtCbIeYQTz+pz7dw67dBQHU9yKX40x+iR813KCgt"
    )
)


# 102: 8.675 4.65
# 103: 11.649 4.64
# 104: 14.12 4.612
# 105: 17.061 4.609
# 106: 19.997 4.568

def get_command(x, y):
    return """tmux send-keys -t "robots_local.2" C-c C-m 'roslaunch rasberry_navigation rasberry_localisation_multisim.launch use_imu:="$USE_IMU" publish_tf:="$EKF_PUBLISH_TF" robot_name:=$ROBOT_NAME_1 initial_pose_x:={:0.3f} initial_pose_y:={:0.3f} initial_pose_a:=$ROBOT_POS_A_1' C-m""".format(
        x, y
    )


LOG_PATH = "."


class Experiment:
    def __init__(self, config):
        self.config = config

        rospy.loginfo(
            "EXP: Initializing Experiment: {}".format(config["experiment_label"])
        )
        self.launch_files = [
            "/home/4D6/einalex/Projekte/workspaces/catkin/src/rasberry_hri/launch/picker_mover.launch",
            "/home/4D6/einalex/Projekte/workspaces/catkin/src/rasberry_hri/launch/hri_agent.launch"
        ]
        self.set_model_state = self.create_service_proxy(
            "/gazebo/set_model_state", SetModelState, local=False
        )
        self.set_robot_position()
        self.is_finished = False
        self.robco = RobotControl(self.config["robot"])
        rospy.loginfo("EXP: Initialization finished")

    def set_robot_position(self):
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        x = self.config["robot_position"][0]  # 14.12 #11.649
        y = self.config["robot_position"][1]  # 4.63 #4.64
        rospy.wait_for_service("/gazebo/set_model_state")
        state_msg = ModelState()
        state_msg.model_name = self.config["robot"]
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.orientation.w = 1
        state_msg.reference_frame = "map"
        rospy.loginfo("Setting robot to {:.2f}, {:.2f}".format(x, y))
        self.set_model_state(state_msg)
        rospy.sleep(1)
        # self.ssh_client.connect("simulator", username="rasberry")
        # stdin, stdout, stderr = self.ssh_client.exec_command(get_command(x, y))
        subprocess.call(get_command(x, y), shell=True)
        # self.ssh_client.close()
        rospy.sleep(3)

    def create_service_proxy(self, topic, type=Empty, local=True):
        if local:
            topic = "/{:}/{:}".format(self.config["robot"], topic)
        rospy.loginfo("EXP: Waiting for service {}".format(topic))
        service = rospy.ServiceProxy("{:}".format(topic), type)
        rospy.wait_for_service(topic)
        rospy.loginfo("EXP: Found service {}".format(topic))
        return service

    def initial_pose_callback(self, msg):
        rospy.loginfo("EXP: Got initial pose")
        rospy.loginfo(msg)

    def launch_services(self):
        run_id = roslaunch.rlutil.get_or_generate_uuid(None, True)
        self.launch = roslaunch.parent.ROSLaunchParent(
            run_id, self.launch_files, is_core=False
        )
        self.launch.start()

    def spin(self):
        """
        Blocks until ROS node is shutdown. Yields activity to other threads.
        @raise ROSInitException: if node is not in a properly initialized state
        """

        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException(
                "client code must call rospy.init_node() first"
            )
        rospy.logdebug(
            "node[%s, %s] entering spin(), pid[%s]",
            rospy.core.get_caller_id(),
            rospy.core.get_node_uri(),
            os.getpid(),
        )
        try:
            while not rospy.core.is_shutdown() and not self.is_finished:
                self.launch.spin_once()
                rospy.rostime.wallsleep(0.01)
            rospy.logwarn("EXP: at end of spin")
        except KeyboardInterrupt:
            rospy.logwarn("EXP: keyboard interrupt")
        except rospy.ROSInterruptException:
            rospy.logwarn("EXP: remote interrupt")
        finally:
            self.shutdown()

    def set_parameters(self):
        for key, value in self.config.items():
            key = "{}/{}".format(NS, key)
            rospy.loginfo("Set parameter: {}:{}".format(key, value))
            rospy.set_param(key, value)
        param_file = os.path.join(CONFIG_DIRECTORY, STATE_DIRECTORY, self.config["run_id"] + ".param")
        rosparam.dump_params(param_file, NS, verbose=False)

    def unset_parameters(self):
        rospy.delete_param("{}/start_time".format(NS))
        for key in self.config.keys():
            try:
                key = "{}/{}".format(NS, key)
                rospy.delete_param(key)
            except KeyError:
                pass

    def setup(self):
        try:
            rospy.delete_param("{}/start_time".format(NS))
        except KeyError:
            pass
        self.set_parameters()
        key = "{}/{}".format(NS, "behaviours")
        rospy.set_param(key, BEHAVIOURS[self.config["behaviour"]])
        self.launch_services()
        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)
        rospy.loginfo("EXP: Experiment setup finished")

        # TODO: wait for everything to have started
        self.start_clock = int(rospy.get_time() + 600)
        running_nodes = rosnode.get_node_names()
        while ("/thorvald_001/picker_mover" not in running_nodes
            or '/thorvald_001/action_recognition_node' not in running_nodes
            or '/thorvald_001/scheduler' not in running_nodes
            or '/thorvald_001/openpose' not in running_nodes):
            rospy.sleep(1)
            running_nodes = rosnode.get_node_names()
        self.start_clock = int(rospy.get_time() + EXPERIMENT_START_DELAY)
        rospy.set_param("{}/start_time".format(NS), self.start_clock)
        rospy.loginfo(
            "EXP: Experiment starting at: {}".format(self.start_clock)
        )

    def clock_callback(self, msg):
        if msg.clock.nsecs == 0:
            clock = msg.clock.to_sec()
            if clock >= self.config["termination_time"] + self.start_clock:
                if not self.is_finished:
                    rospy.loginfo("EXP: Timeout condition reached")
                    self.is_finished = True
                return

    def shutdown(self):
        rospy.loginfo("EXP: Shutting down")
        self.set_model_state.close()
        self.clock_sub.unregister()
        self.launch.shutdown()
        self.robco.cancel_movement()
        self.unset_parameters()
        rospy.loginfo("EXP: Experiment ended")
