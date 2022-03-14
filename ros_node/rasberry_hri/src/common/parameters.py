import random
import bisect
from math import pi

import rospy
ROBOT_NS = rospy.names.get_namespace()
NS = "{}hri".format(ROBOT_NS)
ME = ROBOT_NS.strip("/") or 'meep'

# ME = 'meep'

# the navigation system doesn't stop when requested, add some buffer space
ROBOT_REACTION_SAFETY_MARGIN_STANDING = 0.23
ROBOT_REACTION_SAFETY_MARGIN_MOVING = 0.5

# wait a bit before starting the experiment
EXPERIMENT_START_DELAY = 8  # s


def define(key, default):
    key_path = "{}/{}".format(NS, key)
    # rospy.set_param(key_path, default)
    if rospy.has_param(key_path):
        return rospy.get_param(key_path, default)
    else:
        rospy.set_param(key_path, default)
        return default

# Do Not Change Anything Above This Line
# --------------------------------------------------------------


VERBOSE = False

EXPERIMENT_LABEL = define("experiment_label", "NO LABEL SET")
EXPERIMENT_ID = define("experiment_id", "NO ID SET")
RUN_ID = define("run_id", "NO ID SET")

CONFIG_DIRECTORY = define(
    "config_folder", "/home/4D6/einalex/Projekte/PhD/exp-results")
ACTIVE_DIRECTORY = define("active_folder", "active")
LOG_DIRECTORY = define("log_folder", "log")
STATE_DIRECTORY = define("state_folder", "state")

USE_ACTION_RECOGNITION = define("use_action_recognition", True)

# DISABLE ABILITIES

SIMPLE_MODE = define("simple_mode", False)

GESTURE_PERCEPTION = define("gesture_perception", True)
BEHAVIOR_PERCEPTION = define("behavior_perception", True)
DIRECTION_PERCEPTION = define("direction_perception", True)

ROBOT_CRATE_POSSESSION_PERCEPTION = define(
    "robot_crate_possession_perception", True)
PICKER_CRATE_POSSESSION_PERCEPTION = define(
    "picker_crate_possession_perception", True)
PICKER_CRATE_FILL_PERCEPTION = define("picker_crate_fill_perception", True)

BERRY_POSITION_PERCEPTION = define("berry_position_perception", True)


# PERCEPTION NOISE

# rotation -> rotation, translation -> rotation, translation -> translation, rotation -> translation
MOVEMENT_NOISE = define("movement_noise", [0, 0])
ADD_MOVEMENT_NOISE = MOVEMENT_NOISE != [0, 0]
# TODO: movement noise addition?


class WeightedRandomGenerator(object):
    def __init__(self, weighted_classes):
        self.totals = []
        self.labels = []
        running_total = 0

        for label, w in weighted_classes.items():
            running_total += w
            self.totals.append(running_total)
            self.labels.append(label)

    def next(self):
        rnd = random.random() * self.totals[-1]
        return self.labels[bisect.bisect_right(self.totals, rnd)]

    def __call__(self):
        return self.next()


POSTURE_NOISE = define("posture_noise", 0)
ADD_POSTURE_NOISE = POSTURE_NOISE != 0
# ADD_POSTURE_NOISE = POSTURE_NOISE != 0
# including neutral
# POSTURE_CONFUSION = {"calling": WeightedRandomGenerator({"gesture forward": (1-0.57*(1-POSTURE_NOISE))/0.43 * 0.07, "neutral": (1-0.57*(1-POSTURE_NOISE))/0.43 * 0.34, "calling": (1-POSTURE_NOISE) * 0.57}),
#                      "gesture stop": WeightedRandomGenerator({"picking berries": (1-0.26*(1-POSTURE_NOISE))/0.74 * 0.01, "picking berries right": (1-0.26*(1-POSTURE_NOISE))/0.74 * 0.02, "calling": (1-0.26*(1-POSTURE_NOISE))/0.74 * 0.06, "gesture cancel": (1-0.26*(1-POSTURE_NOISE))/0.74 * 0.07, "gesture forward": (1-0.26*(1-POSTURE_NOISE))/0.74 * 0.12, "gesture backward": (1-0.26*(1-POSTURE_NOISE))/0.74 * 0.13, "gesture stop": (1-POSTURE_NOISE) * 0.26, "neutral": (1-0.26*(1-POSTURE_NOISE))/0.74 * 0.34}),
#                      "gesture cancel": WeightedRandomGenerator({"calling": (1-0.61*(1-POSTURE_NOISE))/0.39 * 0.01, "gesture forward": (1-0.61*(1-POSTURE_NOISE))/0.39 * 0.01, "gesture backward": (1-0.61*(1-POSTURE_NOISE))/0.39 * 0.01, "neutral": (1-0.61*(1-POSTURE_NOISE))/0.39 * 0.36, "gesture cancel": (1-POSTURE_NOISE) * 0.61}),
#                      "gesture forward": WeightedRandomGenerator({"calling": (1-0.7*(1-POSTURE_NOISE))/0.3 * 0.02, "neutral": (1-0.7*(1-POSTURE_NOISE))/0.3 * 0.28, "gesture forward": (1-POSTURE_NOISE) * 0.7}),
#                      "gesture backward": WeightedRandomGenerator({"neutral": (0.29+0.71*POSTURE_NOISE), "gesture backward": (1-POSTURE_NOISE) * 0.71}),
#                      "picking berries": WeightedRandomGenerator({"gesture backward": (1-0.85*(1-POSTURE_NOISE))/0.15 * 0.01, "neutral": (1-0.85*(1-POSTURE_NOISE))/0.15 * 0.02, "gesture cancel": (1-0.85*(1-POSTURE_NOISE))/0.15 * 0.02, "picking berries right": (1-0.85*(1-POSTURE_NOISE))/0.15 * 0.04, "gesture forward": (1-0.85*(1-POSTURE_NOISE))/0.15 * 0.06, "picking berries": (1-POSTURE_NOISE) * 0.85})}

POSTURE_CONFUSION = {
    "calling": WeightedRandomGenerator({
        "calling": (1-POSTURE_NOISE) * 0.87,
        "gesture cancel": (1-0.13*(1-POSTURE_NOISE))/0.87 * 0.01,
        "gesture forward": (1-0.13*(1-POSTURE_NOISE))/0.87 * 0.11,
        "picking berries right": (1-0.13*(1-POSTURE_NOISE))/0.87 * 0.01
    }),
    "gesture cancel": WeightedRandomGenerator({
        "calling": (1-0.21*(1-POSTURE_NOISE))/0.79 * 0.01,
        "gesture cancel": (1-POSTURE_NOISE) * 0.79,
        "picking berries": (1-0.21*(1-POSTURE_NOISE))/0.79 * 0.16,
        "gesture forward": (1-0.21*(1-POSTURE_NOISE))/0.79 * 0.01,
        "gesture backward": (1-0.21*(1-POSTURE_NOISE))/0.79 * 0.02,
        "picking berries right": (1-0.21*(1-POSTURE_NOISE))/0.79 * 0.02
    }),
    "picking berries": WeightedRandomGenerator({
        "gesture cancel": (1-0.13*(1-POSTURE_NOISE))/0.87 * 0.02,
        "picking berries": (1-POSTURE_NOISE) * 0.87,
        "gesture forward": (1-0.13*(1-POSTURE_NOISE))/0.87 * 0.06,
        "gesture backward": (1-0.13*(1-POSTURE_NOISE))/0.87 * 0.01,
        "picking berries right": (1-0.13*(1-POSTURE_NOISE))/0.87 * 0.04
    }),
    "gesture forward": WeightedRandomGenerator({
        "calling": (1-0.03*(1-POSTURE_NOISE))/0.97 * 0.02,
        "gesture forward": (1-POSTURE_NOISE) * 0.97
    }),
    "gesture backward": WeightedRandomGenerator({
        "gesture backward": 1
    })
}


# REASONING

REASONING_LOOP_FREQUENCY = 20  # Hz
ITERATIONS = define("iterations", "30")
COMPLEXITY_PENALTY = define("complexity_penalty", 0.1)
TRUTH_THRESHOLD = 0.5

INF = float("inf")
TARGET = "target"


# BDI

MAX_COST = define("max_cost", 600)
MIN_GAIN = define("min_gain", 0)

# ACTION RECOGNITION

# realsense color raw
CAMERA_TOPIC = define("camera", "/camera/color/image_raw")
# CAMERA_TOPIC = define("camera", "/camera/depth/image_rect_raw")  # realsense color rect
# CAMERA_TOPIC = define("camera", "/camera/infra1/image_rect_raw") # realsense infrared 1
# CAMERA_TOPIC = define("camera", "/camera/infra2/image_rect_raw") # realsense infrared 2
# CAMERA_TOPIC = define("camera", "/zed/right/image_rect_color") # zed stereo right
# CAMERA_TOPIC = define("camera", "/zed/left/image_rect_color") # zed stereo left
# CAMERA_TOPIC = define("camera", "/optris/thermal_image_view") # optris thermal
CLASSIFICATION_TOLERANCE = define("classification_tolerance", 20)
ANGLE_WEIGHT = define("angle_weight", 1.0)
DETECTION_COUNT = define("detection_count", 10)
COOLDOWN = define("cooldown", 0)


# PICKER SETUP

PICKERS = define("pickers", [
    "picker01", "picker02", "picker03", "picker04", "picker05",
    "picker06", "picker07", "picker08", "picker09", "picker10"])

PICKER_LENGTH = 0.4
PICKER_WIDTH = 0.5

PICKER_SPEED = define("picker_speed", 0.7)  # m/s

PICKER_UPDATE_FREQUENCY = define("picker_sim_frequency", 30)  # Hz

PICKER_DISTANCE_PREFERENCE = define("picker_distance", 0.5)


# ROBOT SETUP

ROBOT_LENGTH = 1.5
ROBOT_WIDTH = 1.35584
ROBOT_SPEED = define("robot_speed", 0.7)
ROBOT_POSITION = define("robot_position", [5.53, 4.62])
MINIMUM_DISTANCE = define("minimum_distance", 0.1)  # m
CRATE_CAPACITY = define("crate_capacity", 2)
ROBOT_MODES = define("robot_modes", ["standing", "moving"])

# EXPERIMENT CONFIGURATION

INITIAL_WAYPOINT = define("initial_robot_waypoint", "WayPoint104")
FULL_CRATE_COUNT = define("full_crate_count", "NOT SET")
EMPTY_CRATE_COUNT = define("empty_crate_count", "NOT SET")
TARGET_PICKER = define("target_picker", "NOT SET")

INITIAL_PICKER_POSITION = define(
    "picker_position", [17.561, 4.609])  # [19.997, 4.568])
INITIAL_PICKER_ORIENTATION = define("picker_orientation", 2*pi)
CALLED_ROBOT = define("called_robot", False)
DISMISSED_ROBOT = define("dismissed_robot", False)
SEEN_PICKING = define("seen_picking", False)
HAS_CRATE = define("has_crate", False)
CRATE_FULL = define("crate_full", False)
PICKER_CRATE_FILL_STATE = define("crate_fill_state", 0.5)
PICKER_GESTURE_HISTORY = define("gesture_history", [True, False, True])
NO_BERRY_PLACES = define("no_berry_places", [])


# WORLD SETUP

MEAN_WAYPOINT_DISTANCE = define("mean_waypoint_distance", 2.95)  # m

DEPOT = define("depot_waypoint", "WayPoint69")
READY_POINT = define("ready_waypoint", "WayPoint133")


# ROBOT ACTION PARAMETERS

DEPOSIT_GAIN = define("deposit_gain", 360)
DEPOSIT_COST = define("deposit_cost", 2.5)  # s

GIVE_GAIN = define("give_gain", 70)
GIVE_COST = define("give_cost", 3.5)  # s

EXCHANGE_GAIN = define("exchange_gain", 60)
EXCHANGE_COST = define("exchange_cost", 5)  # s

APPROACH_GAIN = define("approach_gain", 50)

# deposit gain at half capacity: 40

EVADE_GAIN = define("evade_gain", 30)

STANDBY_GAIN = define("standby_gain", 20)

WAIT_GAIN = define("wait_gain", 10)
WAIT_TIME = define("wait_time", 0.5)  # s

# deposit gain at full capacity: 0

MOVE_GAIN = define("move_gain", 0)

MOVETO_GAIN = define("moveto_gain", 0)

TIMEOUT_LENGTH = 60  # s

# PICKER BEHAVIOURS

BEHAVIOURS = {
    "deliver standard": [
        ["call robot", 0],
        ["wait for robot to move", 0],
        ["approach without crate", 0],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "deliver no call": [
        ["expect service", 0],
        ["approach without crate", 0],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "deliver expect robot to come": [
        ["call robot", 0],
        ["wait for robot to arrive", 0],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "deliver expect robot to come simple": [
        ["call robot simple", 0],
        ["wait for robot to arrive", 0],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "deliver expect mind read": [
        ["expect service", 0],
        ["wait for robot to arrive", 0],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "exchange standard": [
        ["pick berries", 4],
        ["call robot", 0],
        ["wait for robot to move", 0],
        ["approach with crate", 0],
        ["return crate", 4],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "exchange no call": [
        ["pick berries", 4],
        ["expect service", 0],
        ["approach with crate", 0],
        ["return crate", 4],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "exchange expect robot to come": [
        ["pick berries", 4],
        ["call robot", 0],
        ["wait for robot to arrive", 0],
        ["return crate", 2.5],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "exchange expect robot to come simple": [
        ["pick berries", 4],
        ["call robot simple", 0],
        ["wait for robot to arrive", 0],
        ["return crate", 2.5],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "exchange expect mind read": [
        ["pick berries", 4],
        ["expect service", 0],
        ["wait for robot to arrive", 0],
        ["return crate", 4],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "call": [
        ["call robot", 0],
        ["expect service", 0],
        ["wait for robot to arrive", 0]
    ],
    "do nothing": [["wait for robot to arrive", 0]],
    "wait": [["wait for robot to arrive", 10]],
    "picking berries": [["pick berries", 4]],
    "evade standard": [["pick berries", 4],
                       ["pass with crate", 0],
                       ["pick berries", 4]]}
