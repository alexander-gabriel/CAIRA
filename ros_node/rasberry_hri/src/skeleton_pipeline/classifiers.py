import rospy
from math import pi

from numpy import arctan2, abs, mean

from common.parameters import *
from common.utils import suppress
from poses import pose_list
from rasberry_hri.msg import Classification, Pose, Criterium

class MinimumDifferenceClassifier:

    def __init__(self):
        self.limit = CLASSIFICATION_TOLERANCE
        self.angle_weight = ANGLE_WEIGHT

    def classify(self, angles, positions):
        errors = dict()
        classification = Classification()
        classification.poses = list()
        min_error = ('Initial', 1000000)
        rospy.logdebug("CLA: -----------------")
        for name, pose in pose_list.items():
            classification.poses.append(Pose())
            classification.poses[-1].label = name
            classification.poses[-1].criteria = list()
            error_list = [self.limit]
            rospy.logdebug("CLA: checking pose: {}".format(name))
            for label, targets in pose.items():
                for target in targets:
                    value = None
                    limit = None
                # with suppress(TypeError):
                    rospy.logdebug("CLA: target: {}:{}".format(label, target))
                    angle_or_position, direction, limit = target.split(" ")
                    if angle_or_position == "a":
                        try:
                            limit = float(limit)
                        except:
                            limit = angles[limit]
                        value = angles[label]
                        if limit > 90 and value <=0:
                            value += 180
                        probability = positions[joint]["P"]
                    else:
                        # if positions[label] is None:
                        #     error_list.append(self.limit)
                        #     break
                        try:
                            limit = float(limit)
                        except:
                            joint, dimension = limit.split("-")
                            limit = positions[joint][dimension]
                            # old
                            # limit = positions[limit]
                        joint, dimension = label.split("-")
                        value = positions[joint][dimension]
                        probability = positions[joint]["P"]
                        # old
                        # value = positions[label]
                    error = False
                    if limit is None or abs(limit - pi) < 0.001:
                        # if limit is None:
                        #     rospy.logwarn("CLA: DETECTED NONE ERROR: {} {}".format(label, target))
                        # else:
                        #     rospy.logwarn("CLA: DETECTED PI ERROR: {} {}".format(label, target))
                        limit = pi
                        error = True
                    if value is None or abs(value - pi) < 0.001:
                        # if value is None:
                        #     rospy.logwarn("CLA: DETECTED NONE ERROR: {} {}".format(label, target))
                        # else:
                        #     rospy.logwarn("CLA: DETECTED PI ERROR: {} {}".format(label, target))
                        value = pi
                        error = True
                    if not error:
                        # rospy.loginfo("CLA: {}: {:f}".format(label, value))
                        if direction == "=":
                            rospy.logdebug("CLA: error: {}-{}={}".format(limit, value, abs(limit - value)))
                            error_list.append(abs(limit - value))
                        elif direction == ">":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, value, abs(limit - value)))
                            error_list.append(max(0, limit - value))
                        elif direction == "<":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, value, abs(value - limit)))
                            error_list.append(max(0, value - limit))
                        else:
                            rospy.logerr("CLA: wrong direction code {}".format(direction))
                            pass
                    # else:
                    #     error_list.append(self.limit)
                    criterium = Criterium()
                    criterium.code = "Angle: {} {}".format(label, target) if angle_or_position == "a" else "Position: {} {}".format(label, target)
                    criterium.limit = limit
                    criterium.value = value
                    criterium.error = error_list[-1]
                    classification.poses[-1].criteria.append(criterium)
            # error = mean(error_list)
            error = mean(error_list)
            classification.poses[-1].error_score = error
            if error < min_error[1]:
                min_error = (name, error)
            rospy.logdebug("CLA: {:}: {:f}".format(name, error))
        classification.label = min_error[0]
        classification.error_score = min_error[1]
        # rospy.loginfo("CLA: -----------------")
        # rospy.loginfo("CLA: {:}: {:f}".format(min_error[0], min_error[1]))
        return min_error, classification
