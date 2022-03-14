import os
from threading import Lock
import collections

from math import sqrt, sin, cos, atan2
from functools import partial
# from profilehooks import profile, coverage

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace

import numpy as np


from opencog.type_constructors import ConceptNode
from opencog.utilities import initialize_opencog

import tf
import rospy
from rasberry_hri.msg import Action
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from common.utils import wp2sym, db, suppress, atomspace

from bdi_system import BDISystem
from knowledge_base import KnowledgeBase

from common.parameters import (
    NS, MINIMUM_DISTANCE, REASONING_LOOP_FREQUENCY,
    PICKER_WIDTH, PICKER_LENGTH, ROBOT_WIDTH, ROBOT_LENGTH, PICKER_SPEED,
    PICKER_UPDATE_FREQUENCY, DIRECTION_PERCEPTION, PICKERS,
    ROBOT_REACTION_SAFETY_MARGIN_MOVING, ROBOT_REACTION_SAFETY_MARGIN_STANDING,
    MOVEMENT_NOISE, TARGET_PICKER)
QUANTIZATION = PICKER_SPEED / PICKER_UPDATE_FREQUENCY * 0.9


# from opencog.logger import log

# log.use_stdout()
# log.set_level("DEBUG")


class Scheduler:

    def __init__(self, robot_id):
        rospy.loginfo("SCH: Initializing Scheduler")
        self.shutting_down = False
        self.rate = rospy.Rate(REASONING_LOOP_FREQUENCY)
        self.atomspace = atomspace
        self.kb = KnowledgeBase()
        self.sensory_lock1 = Lock()
        self.sensory_lock2 = Lock()
        self.has_reached_50cm = {}
        self.has_reached_100cm = {}
        self.has_reached_150cm = {}
        self.has_reached_200cm = {}
        self.latest_robot_msg = None
        self.latest_actual_robot_msg = None
        self.latest_people_msgs = {}
        self.latest_actual_people_msgs = {}
        self.robot_tracks = {}
        self.people_tracks = {}
        self.directions = {}
        self.latest_people_nodes = {}
        self.latest_distances = {}
        self.qsrlib = QSRlib()
        self.speed = [-1, -1, -1, -1]
        self.human_position_subs = []
        self.robot_id = robot_id
        self.latest_robot_node = None
        self.bdi = BDISystem(self.robot_id, self.kb)
        rospy.on_shutdown(self.shutdown)
        for name in PICKERS:
            self.has_reached_50cm[name] = False
            self.has_reached_100cm[name] = False
            self.has_reached_150cm[name] = False
            self.has_reached_200cm[name] = False
            rospy.loginfo("SCH: Subscribing to /{}/posestamped".format(name))
            self.human_position_subs.append(rospy.Subscriber(
                "/{}/posestamped".format(name),
                PoseStamped,
                partial(self.human_position_callback, name=name)
            ))
        self.robot_pose_sub = rospy.Subscriber(
            "/{:}/robot_pose".format(self.robot_id),
            Pose,
            self.robot_position_coordinate_callback,
        )
        self.robot_sub = rospy.Subscriber(
            "/{:}/closest_node".format(self.robot_id),
            String,
            self.robot_position_node_callback,
        )
        self.human_action_sub = rospy.Subscriber(
            "{}/human_actions".format(NS),
            Action, self.human_action_callback
        )
        rospy.loginfo("SCH: Initialization finished")

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
            while not rospy.core.is_shutdown():
                self.bdi.loop()
                self.rate.sleep()
            rospy.loginfo("SCH: at end of spin")
        except KeyboardInterrupt:
            rospy.logwarn("SCH: keyboard interrupt")
        except rospy.ROSInterruptException:
            rospy.logwarn("SCH: remote interrupt")
        finally:
            self.shutdown()

    def shutdown(self):
        if not self.shutting_down:
            rospy.loginfo("SCH: Shutting down")
            self.shutting_down = True
            self.bdi.shutdown()
            self.robot_pose_sub.unregister()
            self.robot_sub.unregister()
            self.human_action_sub.unregister()
            for sub in self.human_position_subs:
                sub.unregister()

    def add_position_noise(self, pose, old_pose):
        if old_pose is not None:
            dx = pose.position.x - old_pose.position.x
            dy = pose.position.y - old_pose.position.y
            translation = sqrt(dx*dx + dy*dy)
            q = [old_pose.orientation.x,
                 old_pose.orientation.y,
                 old_pose.orientation.z,
                 old_pose.orientation.w]
            (r, p, old_theta) = tf.transformations.euler_from_quaternion(q)
            q = [pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w]
            (r, p, theta) = tf.transformations.euler_from_quaternion(q)
            pre_movement_rotation = atan2(dy, dx) - old_theta
            post_movement_rotation = theta - (pre_movement_rotation
                                              + old_theta)
            # calculate standard deviations
            sd_pre_movement_rotation = (
                MOVEMENT_NOISE[0] * abs(pre_movement_rotation)
                + MOVEMENT_NOISE[1] * translation)
            sd_post_movement_rotation = (
                MOVEMENT_NOISE[0] * abs(post_movement_rotation)
                + MOVEMENT_NOISE[1] * translation)
            sd_translation = (
                MOVEMENT_NOISE[2] * translation
                + MOVEMENT_NOISE[3] * (abs(pre_movement_rotation)
                                       + abs(post_movement_rotation)))

            translation += np.random.normal(
                0, sd_translation * sd_translation)
            pre_movement_rotation += np.random.normal(
                0, sd_pre_movement_rotation * sd_pre_movement_rotation)
            post_movement_rotation += np.random.normal(
                0, sd_post_movement_rotation * sd_post_movement_rotation)

            pose.position.x = old_pose.position.x + \
                translation * cos(old_theta + pre_movement_rotation)
            pose.position.y = old_pose.position.y + \
                translation * sin(old_theta + pre_movement_rotation)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, \
                pose.orientation.w = tf.transformations.quaternion_from_euler(
                    0, 0, old_theta + pre_movement_rotation
                    + post_movement_rotation)
        return pose

    def get_speed(self, positions):
        # for position in positions:
        #     rospy.loginfo(position)
        # simple speed estimation from last two log entries
        pos_1 = positions[-1].to_list()
        x1 = pos_1[0]
        y1 = pos_1[1]
        t1 = pos_1[2]
        pos_2 = positions[-2].to_list()
        x2 = pos_2[0]
        y2 = pos_2[1]
        t2 = pos_2[2]
        v = sqrt((x1-x2)**2 + (y1-y2)**2)/(t1-t2)
        return v

    def robot_position_coordinate_callback(self, pose):
        # rospy.loginfo("SCH: Robot position coordinate callback")
        self.latest_actual_robot_msg = pose
        self.latest_robot_msg = pose
        self.bdi.world_state.set_position(
            self.bdi.me, pose.position.x, pose.position.y, rospy.get_time())

    def robot_position_node_callback(self, msg):
        # rospy.loginfo("SCH: Robot position node callback")
        if msg.data != "none":
            initialize_opencog(self.atomspace)
            self.latest_robot_node = wp2sym(msg.data)
            self.bdi.world_state.update_position(
                ConceptNode(self.robot_id),
                ConceptNode(self.latest_robot_node),
            )
        else:
            self.latest_robot_node = None

    def human_position_callback(self, msg, name):

        timestamp = rospy.get_time()
        initialize_opencog(self.atomspace)
        if not self.sensory_lock2.locked():
            self.sensory_lock2.acquire()
            self._handle_position_msgs(name, msg, timestamp)
            self.sensory_lock2.release()
        if not self.sensory_lock1.locked():
            self.sensory_lock1.acquire()
            person = ConceptNode(name)
            self.bdi.world_state.set_position(
                person, msg.pose.position.x, msg.pose.position.y, timestamp)
            self._update_picker_node(person, msg)
            self.latest_people_msgs[name] = msg
            self._react_to_distance_events(person)
            self.sensory_lock1.release()

    def human_action_callback(self, msg):
        if msg.action != "":
            initialize_opencog(self.atomspace)
            # msg.person = self.get_closest_human()
            self.bdi.world_state.update_action(
                msg.person, msg.action, msg.error_score)

    def get_closest_human(self):
        closest_human = None
        shortest_distance = float("inf")
        for id in self.latest_people_msgs.keys():
            distance = self.bdi.world_state.get_distance(
                ConceptNode(self.robot_id), ConceptNode(id), True)
            if distance < shortest_distance:
                distance = shortest_distance
                closest_human = id
        # rospy.logwarn("Closest picker is: {}".format(closest_human))
        return closest_human

    def _update_picker_node(self, person, msg):
        try:
            (current_node, closest_node) = self.bdi.locator.localise_pose(msg)
            # rospy.logwarn("\ncurrent: {:}\nclosest: {:}"
            #                 .format(current_node, closest_node))
            # if current_node == "WayPoint104":
            #     self.kb.debug += 1
            latest_node = None
            with suppress(KeyError):
                latest_node = self.latest_people_nodes[person.name][1]
            if current_node != "none":
                self.latest_people_nodes[person.name] = ("is_at", current_node)
                if current_node != latest_node:
                    self.bdi.world_state.update_position(
                        person, ConceptNode(current_node)
                    )
            elif closest_node is None:
                rospy.logwarn(
                    ("SCH: We have no idea where {} is currently").format(
                        person.name
                    )
                )
            else:  # we know the closest but not the current node
                self.latest_people_nodes[person.name] = ("is_at", closest_node)
                if closest_node != latest_node:
                    self.bdi.world_state.update_position(
                        person, ConceptNode(closest_node)
                    )
        except TypeError as err:
            rospy.logerr(
                ("SCH - Couldn't update picker node. " "Error: {:}").format(
                    err
                )
            )

    def _react_to_distance_events(self, person):
        try:
            distance = self.bdi.world_state.get_distance(self.bdi.me, person)
            self.latest_distances[person.name] = distance
            minimum_distance = max(
                MINIMUM_DISTANCE,
                self.bdi.world_state.get_optimum_distance(person)
            )
            try:
                if self.directions[person.name][0] == "-":
                    minimum_distance += ROBOT_REACTION_SAFETY_MARGIN_MOVING
                else:
                    minimum_distance += ROBOT_REACTION_SAFETY_MARGIN_STANDING
            except KeyError:
                minimum_distance += ROBOT_REACTION_SAFETY_MARGIN_STANDING
            if self.bdi.world_state.robot_direction == self.bdi.world_state._approaching:
                if distance <= minimum_distance:
                    if not self.bdi.world_state.too_close:
                        rospy.loginfo(
                            "SCH: Robot has met picker. Halting at {:.2f}."
                            .format(distance))
                        self.bdi.world_state.too_close = True
                        self.bdi.robco.cancel_movement()
                        x, y, _ = self.bdi.world_state.get_position(
                            self.bdi.me)[-1].to_list()
                        db.add_meet_entry(distance, self.speed)
            if self.bdi.world_state.robot_direction == self.bdi.world_state._leaving:
                if distance > minimum_distance and self.bdi.world_state.too_close:
                    rospy.loginfo("SCH: Robot has left picker.")
                    self.bdi.world_state.too_close = False
            if distance <= 0.5 \
               and self.has_reached_100cm[person.name] \
               and not self.has_reached_50cm[person.name]:
                self.has_reached_50cm[person.name] = True
                rospy.loginfo("SCH: Distance to {:} is 50cm: {:.2f}"
                              .format(person.name, distance))
                # save half meter distance speed
                speed = self.get_speed(
                    self.bdi.world_state.get_position(self.bdi.me))
                if self.speed[3] == -1:
                    self.speed[3] = speed
            elif (distance <= 1.0
                  and self.has_reached_150cm[person.name]
                  and not self.has_reached_100cm[person.name]):
                self.has_reached_100cm[person.name] = True
                rospy.loginfo("SCH: Distance to {:} is 100cm: {:.2f}"
                              .format(person.name, distance))
                # save one meter distance speed
                speed = self.get_speed(
                    self.bdi.world_state.get_position(self.bdi.me))
                if self.speed[2] == -1:
                    self.speed[2] = speed
            elif (distance <= 1.5
                  and self.has_reached_200cm[person.name]
                  and not self.has_reached_150cm[person.name]):
                self.has_reached_150cm[person.name] = True
                rospy.loginfo("SCH: Distance to {:} is 150cm: {:.2f}"
                              .format(person.name, distance))
                # save two meter distance spe.ed
                speed = self.get_speed(
                    self.bdi.world_state.get_position(self.bdi.me))
                if self.speed[1] == -1:
                    self.speed[1] = speed
            elif (distance <= 2.0
                  and not self.has_reached_200cm[person.name]):
                self.has_reached_200cm[person.name] = True
                rospy.loginfo("SCH: Distance to {:} is 200cm: {:.2f}"
                              .format(person.name, distance))
                # save two meter distance speed
                speed = self.get_speed(
                    self.bdi.world_state.get_position(self.bdi.me))
                if self.speed[0] == -1:
                    self.speed[0] = speed
            elif (distance > 2.1
                  and self.has_reached_200cm[person.name]):
                self.has_reached_50cm[person.name] = False
                self.has_reached_100cm[person.name] = False
                self.has_reached_150cm[person.name] = False
                self.has_reached_200cm[person.name] = False
                self.speed = [-1, -1, -1, -1]
            # self.bdi.last_distance = distance
        except Exception as err:
            rospy.logerr(
                "SCH: 275 - Couldn't react to distance events. Error: {:}"
                .format(err)
            )

    def _calculate_directions(self, world, pairs):
        dynamic_args = {
            "qtcbs": {
                "quantisation_factor": QUANTIZATION,
                "validate": False,
                "no_collapse": False,
                "qsrs_for": pairs,
            }
        }
        qsrlib_request_message = QSRlib_Request_Message(
            which_qsr="qtcbs", input_data=world, dynamic_args=dynamic_args
        )
        try:
            qsrlib_response_message = self.qsrlib.request_qsrs(
                qsrlib_request_message
            )
            t = qsrlib_response_message.qsrs.get_sorted_timestamps()[-1]
            for k, v in qsrlib_response_message.qsrs.trace[t].qsrs.items():
                picker = ConceptNode(k.split(",")[1])
                robot_direction, picker_direction = v.qsr.get(
                    "qtcbs").split(",")
                # picker_direction = directions[1]
                if picker.name == TARGET_PICKER:
                    if robot_direction == "-":
                        self.bdi.world_state.robot_direction = self.bdi.world_state._approaching
                    elif robot_direction == "+":
                        self.bdi.world_state.robot_direction = self.bdi.world_state._leaving
                    else:
                        self.bdi.world_state.robot_direction = self.bdi.world_state._standing
                try:
                    self.directions[picker.name].appendleft(picker_direction)
                    if self.directions[picker.name].count(picker_direction) == 5 and not picker_direction == "0":
                        self._handle_direction_change(picker, picker_direction)
                    elif abs(self.directions[picker.name].count("+") - self.directions[picker.name].count("-")) < 2:
                        self._handle_direction_change(picker, "0")
                except KeyError:
                    self.directions[picker.name] = collections.deque(
                        [picker_direction], maxlen=7)
        except ValueError:
            pass
        except IndexError as err:
            rospy.logwarn("SCH: 414 - Index error: {}".format(err))
        except KeyError as err:
            rospy.logerr("SCH: 416 - Timestamp mismatch error: {}".format(err))

    def _handle_direction_change(self, picker, direction):
        if direction == "+":
            if not (
                self.bdi.world_state.is_picker_movement(
                    picker, self.bdi.world_state._leaving)
            ):
                self.bdi.world_state.set_picker_movement(
                    picker, self.bdi.world_state._leaving)
                rospy.loginfo("SCH: Observation: {} is leaving"
                              .format(picker.name))
        elif direction == "-":
            if not (
                self.bdi.world_state.is_picker_movement(
                    picker, self.bdi.world_state._approaching)
            ):
                self.bdi.world_state.set_picker_movement(
                    picker, self.bdi.world_state._approaching)
                rospy.loginfo(
                    "SCH: Observation: {} is approaching"
                    .format(picker.name)
                )
        else:
            if not (
                self.bdi.world_state.is_picker_movement(
                    picker, self.bdi.world_state._standing)
            ):
                if (
                    self.bdi.world_state.is_picker_movement(
                        picker, self.bdi.world_state._approaching)
                ):
                    self.bdi.world_state.set_latest_distance(
                        picker, self.latest_distances[picker.name]
                    )
                    rospy.loginfo(
                        "SCH: Observation: {} is stopping at {:.2f}m distance"
                        .format(picker.name, self.latest_distances[picker.name])
                    )
                    self.bdi.world_state.set_picker_movement(
                        picker, self.bdi.world_state._standing)
                else:
                    if not (
                        self.bdi.world_state.is_picker_movement(
                            picker, self.bdi.world_state._standing)
                    ):
                        rospy.loginfo(
                            "SCH: Observation: {} is standing"
                            .format(picker.name)
                        )
                        self.bdi.world_state.set_picker_movement(
                            picker, self.bdi.world_state._standing)

    def _handle_position_msgs(self, name, msg, timestamp):
        """Abstracts received position messages to the Knowledge Base"""
        if self.latest_robot_msg is not None:
            world = World_Trace()
            position = Object_State(
                    name=self.robot_id,
                    timestamp=timestamp,
                    x=self.latest_robot_msg.position.x,
                    y=self.latest_robot_msg.position.y,
                    xsize=ROBOT_WIDTH,
                    ysize=ROBOT_LENGTH,
                    object_type="Person"
                )
            try:
                self.robot_tracks[name].append(position)
            except KeyError:
                self.robot_tracks[name] = collections.deque(
                    [position], maxlen=2)
            world.add_object_state_series(self.robot_tracks[name])
            position = Object_State(
                name=name,
                timestamp=timestamp,
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                xsize=PICKER_WIDTH,
                ysize=PICKER_LENGTH,
                object_type="Person"
            )
            try:
                self.people_tracks[name].append(position)
            except KeyError:
                self.people_tracks[name] = collections.deque(
                    [position], maxlen=2)
            world.add_object_state_series(self.people_tracks[name])
            pairs = [(self.robot_id, name)]
            if DIRECTION_PERCEPTION:
                self._calculate_directions(world, pairs)
                # if direction changed and picker close and robot stationary, consider saving new perferred distance

    def save(self):
        self.bdi.save()
