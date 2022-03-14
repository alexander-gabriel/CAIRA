import json
import os
import rospy

from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize
from rasberry_hri.msg import Action, Command, Joint, Classification

from common.utils import get_angle_prototype, get_position_prototype, suppress
from common.parameters import NS, DETECTION_COUNT, COOLDOWN, TARGET_PICKER, \
    CAMERA_TOPIC, BEHAVIOR_PERCEPTION, GESTURE_PERCEPTION, CONFIG_DIRECTORY, \
    LOG_DIRECTORY, USE_ACTION_RECOGNITION, ADD_POSTURE_NOISE, POSTURE_CONFUSION

from converter import Converter
from classifiers import MinimumDifferenceClassifier
from openpose import Openpose


class ActionRecognition:

    def __init__(self):
        self.normal_mode = USE_ACTION_RECOGNITION
        rospy.loginfo("ACR: ActionRecognition Node starting")
        if self.normal_mode:
            self.action_publisher = rospy.Publisher('{}/human_actions'.format(
                                                    NS), Action, queue_size=10)
            rospy.loginfo(
                "ACR: Created publisher at {}/human_actions".format(NS))
        else:
            self.action_publisher2 = rospy.Publisher('/human_actions_fast',
                                                     Action, queue_size=10)
        self.command_publisher = rospy.Publisher('/movement_control',
                                                 Command, queue_size=10)
        self.classification_publisher = rospy.Publisher('/pose_classification',
                                                        Classification,
                                                        queue_size=10)
        # self.picker = TARGET_PICKER
        self.converter = Converter()
        self.classifier = MinimumDifferenceClassifier()
        if self.normal_mode:
            rospy.loginfo("ACR: Waiting for Recognition service")
            rospy.wait_for_service('recognize')
            self.interface = rospy.ServiceProxy('recognize', Recognize)
            rospy.loginfo("ACR: Waiting for OpenPose")
            self.openpose = Openpose(self.interface, self.openpose_callback)
        self.detection_count = DETECTION_COUNT
        self.cooldown = COOLDOWN
        self.last_detected_count = {}
        camera = CAMERA_TOPIC
        if self.normal_mode and (BEHAVIOR_PERCEPTION or GESTURE_PERCEPTION):
            self.sub = rospy.Subscriber(camera, Image, self.callback_rgb)
            rospy.loginfo("ACR: Subscribed to {:}".format(camera))
        elif not self.normal_mode:
            self.sub = rospy.Subscriber(
                '{}/human_actions'.format(NS), Action, self.action_callback)
            rospy.loginfo("ACR: Subscribed to {}/human_actions".format(NS))
        rospy.loginfo("ACR: Initialization Complete")

    def run(self):
        try:
            if self.normal_mode:
                self.openpose.run()
                rospy.logwarn("ACR: at end of openpose")
            else:
                rospy.spin()
                rospy.logwarn("ACR: at end of spin")
        except rospy.ROSInterruptException:
            rospy.logwarn("ACR: remote interrupt")
        except Exception as err:
            rospy.logwarn("ACR: unknown exception")
            rospy.logerr(err)
        finally:
            rospy.logwarn("ACR: we reached finally")
            self.shutdown()

    def openpose_callback(self, timestamp, source, response):
        if response.recognitions:
            self.converter.create_index_map(response.recognitions)
            angles = self.get_angles()
            positions = self.get_positions()
            with open(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, "angles.log"), 'w') as f:
                json.dump(angles, f)
            with open(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, "positions.log"), 'w') as f:
                json.dump(positions, f)
            action, classification = self.get_action(angles, positions)
            classification.header.stamp = timestamp
            self.classification_publisher.publish(classification)
            if GESTURE_PERCEPTION and action in [
                    "calling", "gesture cancel", "gesture stop",
                    "gesture forward", "gesture backward"] \
               or BEHAVIOR_PERCEPTION and action in [
                    "picking berries", "picking berries right",
                    "put or get crate"] \
               or action == "neutral":
                with open(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, "action.log"), 'w') as f:
                    json.dump(action, f)
                if action is not None:
                    # rospy.loginfo("ACR: Detected behavior '{}'".format(action))
                    outmsg = Action()
                    outmsg.header.stamp = timestamp
                    outmsg.error_score = classification.error_score
                    outmsg.action = action or ""
                    # define("target_picker","PICKER_NAME_NOT_SET") #self.picker
                    outmsg.person = TARGET_PICKER
                    # outmsg.header.stamp = rospy.get_rostime()
                    outmsg.joints = []
                    for key in angles.keys():
                        joint = Joint()
                        joint.label = key
                        try:
                            joint.angle = angles[key]
                        except Exception:
                            pass
                        try:
                            joint.position.x = positions[key]["X"]
                            joint.position.y = positions[key]["Y"]
                        except Exception:
                            pass
                        try:
                            joint.confidence = positions[key]["P"]
                        except Exception:
                            pass
                        outmsg.joints.append(joint)
                        # TODO: identify picker

                    self.action_publisher.publish(outmsg)
                    rospy.logdebug("ACR: Published behavior '{}: {}'".format(
                        outmsg.person, action))
                    # outmsg = Command()
                    # outmsg.header.stamp = timestamp
                    # outmsg.command = action
                    # self.command_publisher.publish(outmsg)

    def action_callback(self, msg):
        # start_time = time.time()
        positions = get_position_prototype()
        angles = get_angle_prototype()
        for joint in msg.joints:
            angles[joint.label] = joint.angle
            positions[joint.label] = {}
            positions[joint.label]['X'] = joint.position.x
            positions[joint.label]['Y'] = joint.position.y
            positions[joint.label]['P'] = 0.5  # joint.confidence
        with open(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, "angles.log"), 'w') as f:
            json.dump(angles, f)
        with open(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, "positions.log"), 'w') as f:
            json.dump(positions, f)
        action, classification = self.get_action(angles, positions)
        classification.header.stamp = msg.header.stamp
        self.classification_publisher.publish(classification)
        with open(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY) + "action.log", 'w') as f:
            json.dump(action, f)
        if action is not None:
            rospy.logdebug("ACR: Detected behavior '{}'".format(action))
            outmsg = Action()
            outmsg.header.stamp = msg.header.stamp
            outmsg.action = action or ""
            # define("target_picker","PICKER_NAME_NOT_SET")
            outmsg.person = TARGET_PICKER
            # outmsg.header.stamp = rospy.get_rostime()
            outmsg.joints = []
            for key in angles.keys():
                key = key[:-2]

                joint = Joint()
                joint.label = key
                try:
                    joint.angle = angles[key]
                except Exception:
                    pass
                try:
                    joint.position.x = positions[key]["X"]
                    joint.position.y = positions[key]["Y"]
                except Exception:
                    pass
                try:
                    joint.confidence = positions[key]["P"]
                except Exception:
                    pass
                outmsg.joints.append(joint)
                # TODO: identify picker
                # rospy.logwarn("ACR: action recognition took {:.4f}s".format(time.time()-start_time))
                self.action_publisher2.publish(outmsg)

    def get_positions(self):
        # convert x/y offsets to values relative to some reference point (neck?)
        # adjust joint number and setup and labels
        model = get_position_prototype()
        for id in model.keys():
            with suppress(KeyError):
                model[id] = self.converter.get_position(id)
        return model

    def get_angles(self):
        model = get_angle_prototype()
        for id in ['Neck', 'Upper-Spine', 'Mid-Spine', 'Lower-Spine']:
            try:
                model[id] = self.converter.get_angle2(id)
            except Exception:
                pass
        for id in ['Right:Elbow', 'Upper-Spine', 'Mid-Spine', 'Lower-Spine',
                   'Right:Shoulder', 'Right:Hip', 'Right:Knee']:
            try:
                model[id] = self.converter.get_angle1(id)
            except Exception:
                pass
        for id in ['Left:Elbow', 'Left:Shoulder', 'Left:Hip', 'Left:Knee']:
            try:
                model[id] = self.converter.get_angle1(id)
            except Exception:
                pass
        return model

    def get_action(self, angles, positions):
        tuple, classification = self.classifier.classify(angles, positions)
        error = tuple[1]
        # decide if noise or not
        if ADD_POSTURE_NOISE:
            try:
                action_label = POSTURE_CONFUSION[tuple[0]]()
                rospy.loginfo("ACR: {} using noise".format(action_label))
            except KeyError:
                action_label = tuple[0]
                rospy.loginfo(
                    "ACR: {} failed to use noise".format(action_label))
        else:
            action_label = tuple[0]
            # rospy.loginfo("ACR: {} NOT using noise".format(action_label))
        try:
            rospy.logdebug(
                "ACR: Detected behavior '{}' with penalty {}, counts {}"
                .format(action_label, error,
                        self.last_detected_count[action_label]))
        except Exception:
            rospy.logdebug(
                "ACR: Detected behavior '{}' with penalty {}, counts {}"
                .format(action_label, error, 0))
        if error < self.classifier.limit:
            if action_label not in self.last_detected_count:
                self.last_detected_count[action_label] = 1
            else:
                self.last_detected_count[action_label] += 1
            # TODO: readjust
            if self.last_detected_count[action_label] == self.detection_count:
                self.last_detected_count.clear()
                self.last_detected_count[action_label] = self.cooldown
                return (action_label, classification)
            else:
                return (None, classification)
        else:
            return (None, classification)

    def callback_rgb(self, data):
        self.openpose.latest_rgb.append(data)

    def callback_thermal(self, data):
        self.openpose.latest_thermal.append(data)

    def shutdown(self):
        rospy.loginfo("ACR: Shutting down")
        try:
            self.sub.unregister()
        except AttributeError:
            pass
