from math import pi
from numpy import arctan2, abs
import numpy as np

import rospy

from filter import LimbFilter, PositionFilter
from common.utils import suppress
from common.parameters import POSTURE_NOISE, ADD_POSTURE_NOISE

class Converter:

    def __init__(self):
        self.angle_joints = {'Right:Elbow' : ("RWrist", "RElbow", "RShoulder"),
                             'Left:Elbow' : ("LShoulder", "LElbow", "LWrist"),
                             'Right:Shoulder' : ("Neck", "RShoulder", "RElbow"),
                             'Left:Shoulder' : ("LElbow", "LShoulder", "Neck"),
                             'Right:Knee' : ("RAnkle", "RKnee", "RHip"),
                             'Left:Knee' : ("LAnkle", "LKnee", "LHip"),
                             'Right:Hip' : ("RKnee", "RHip", "LHip"),
                             'Left:Hip' : ("LKnee", "LHip", "RHip"),
                             'Upper-Spine' : ("RHip", "LHip", "Neck"),
                             'Mid-Spine' : ("RHip", "LHip", "Neck"),
                             'Lower-Spine' : ("RHip", "LHip", "Neck"),
                             'Neck' : ("REar", "LEar", "Neck")}

        self.position_table = {}
        self.actual_positions = None
        self.label_table = {'RBigToe': 'Right:BigToe',
                                'LBigToe': 'Left:BigToe',
                                'RSmallToe': 'Right:SmallToe',
                                'LSmallToe': 'Left:SmallToe',
                                'RHeel': 'Right:Heel',
                                'LHeel': 'Left:Heel',
                                'RAnkle': 'Right:Ankle',
                                'LAnkle': 'Left:Ankle',
                                'RKnee': 'Right:Knee',
                                'LKnee': 'Left:Knee',
                                'RHip': 'Right:Hip',
                                'MidHip': 'Mid:Hip',
                                'LHip': 'Left:Hip',
                                'Neck': 'Neck',
                                'RWrist': 'Right:Wrist',
                                'LWrist': 'Left:Wrist',
                                'RElbow': 'Right:Elbow',
                                'LElbow': 'Left:Elbow',
                                'RShoulder': 'Right:Shoulder',
                                'LShoulder': 'Left:Shoulder',
                                'Nose': 'Nose',
                                'REye': 'Right:Eye',
                                'LEye': 'Left:Eye',
                                'REar': 'Right:Ear',
                                'LEar': 'Left:Ear'}
        self.limb_filter = LimbFilter(0.5)
        self.position_filter = PositionFilter()

    def add_pose_noise(self, pose):
        return {
            "X": pose["X"] + np.random.normal(0, POSTURE_NOISE[0]),
            "Y": pose["Y"] + np.random.normal(0, POSTURE_NOISE[1]),
            "P": pose["P"]
        }

    def create_index_map(self, recognitions):
        self.index_map = {}
        self.recognitions = recognitions
        for index, entry in enumerate(self.recognitions):
            if entry.group_id == 0:
                self.index_map[entry.categorical_distribution.probabilities[0].label] = index
        centerX = self.X("Neck")
        centerY = self.Y("Neck")
        def abs2rel(pos):
            if pos.roi.x_offset == 0 and pos.roi.y_offset == 0:
                return {"X": pi,
                        "Y": pi,
                        "P": pos.categorical_distribution.probabilities[0].probability}
            else:
                return {"X": pos.roi.x_offset - centerX,
                        "Y": pos.roi.y_offset - centerY,
                        "P": pos.categorical_distribution.probabilities[0].probability}
        self.positions = {}
        old_positions = self.actual_positions
        self.actual_positions = {}
        for label in self.label_table.keys():
            with suppress(KeyError):
                # position = recognitions[self.index_map[label]].roi
                new_label = self.label_table[label]
                self.actual_positions[new_label] = abs2rel(recognitions[self.index_map[label]])
                try:
                    # if ADD_POSTURE_NOISE:
                    if False:
                        self.positions[new_label] = self.add_pose_noise(self.actual_positions[new_label])
                    else:
                        self.positions[new_label] = self.actual_positions[new_label]
                except Exception:
                    self.positions[new_label] = self.actual_positions[new_label]
                # self.positions[new_label]["X"] = position.x_offset - centerX
                # self.positions[new_label]["Y"] = position.y_offset - centerY
                # self.positions[new_label]["P"] = recognitions[self.index_map[label]].categorical_distribution.probabilities[0].probability

    def X(self, label):
        return self.recognitions[self.index_map[label]].roi.x_offset

    def Y(self, label):
        return self.recognitions[self.index_map[label]].roi.y_offset

    def get_angle1(self, joint_name):
        joints = self.angle_joints[joint_name]
        v0 = np.array([self.X(joints[0]), self.Y(joints[0])]) - np.array([self.X(joints[1]), self.Y(joints[1])])
        v1 = np.array([self.X(joints[2]), self.Y(joints[2])]) - np.array([self.X(joints[1]), self.Y(joints[1])])
        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        angle = np.degrees(angle)
        if angle < 0:
            angle += 360
        return angle

    def get_position(self, joint_name):
        return self.positions[joint_name]

    def get_angle2(self, joint_name):
        joints = self.angle_joints[joint_name]
        midX = (self.X(joints[0]) - self.X(joints[1])) / 2.0
        midY = (self.Y(joints[0]) - self.Y(joints[1])) / 2.0
        v0 = np.array([self.X(joints[0]), self.Y(joints[0])]) - np.array([midX, midY])
        v1 = np.array([self.X(joints[2]), self.Y(joints[2])]) - np.array([midX, midY])

        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        angle = np.degrees(angle)
        if angle < 0:
            angle += 360
        return angle

    def get_angle3(self, joint_name):
        joints = self.angle_joints[joint_name]
        aX = self.X(joints[0]) - self.X(joints[1])
        bX = self.X(joints[2]) - self.X(joints[1])
        cX = aX - bX
        aY = self.Y(joints[0]) - self.Y(joints[1])
        bY = self.Y(joints[2]) - self.Y(joints[1])
        cY = aY - bY
        return arctan2(-cY,cX)

    def get_angle4(self, joint_name):
        joints = self.angle_joints[joint_name]
        midX = (self.X(joints[0]) - self.X(joints[1]))/ 2.0
        midY = (self.Y(joints[0]) - self.Y(joints[1]))/2.0
        cX = self.X(joints[2]) - midX
        cY = self.Y(joints[2]) - midY
        return arctan2(-cY,cX)

    def from_openpose(self, recognitions):
        joints = {}
        recognitions = self.limb_filter.filter(recognitions)
        recognitions = self.position_filter.filter(recognitions)
        for entry in recognitions:
            label = entry.categorical_distribution.probabilities[0].label
            probability = entry.categorical_distribution.probabilities[0].probability
            X = entry.roi.x_offset
            Y = entry.roi.y_offset
            joints[label] = {}
            joints[label]['probability'] = probability
            joints[label]['X'] = X
            joints[label]['Y'] = Y
            joints[label]['Z'] = -1
        return joints

    def from_openpose_labels(self, label):
        return self.label_table[label]
