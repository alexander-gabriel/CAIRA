from numpy import abs

import rospy

class Filter(object):

    def filter(self, recognitions):
        return None



class LimbFilter(Filter):

    def __init__(self, limit):
        self.limit = limit

    def filter(self, recognitions):
        entries = {}
        approved = []
        new_recognitions = []
        for recognition in recognitions:
            if not recognition.group_id in entries:
                entries[recognition.group_id] = {}
            joint = recognition.categorical_distribution.probabilities[0].label
            P = recognition.categorical_distribution.probabilities[0].probability
            X = recognition.roi.x_offset
            Y = recognition.roi.y_offset
            entries[recognition.group_id][joint] = (P, X, Y)
        for id, entry in entries.items():
            if entry["RShoulder"][0] > self.limit and entry["LShoulder"][0] > self.limit and entry["RElbow"][0] > self.limit and entry["LElbow"][0] > self.limit and entry["RWrist"][0] > self.limit and entry["LWrist"][0] > self.limit:
                approved.append(id)
        for recognition in recognitions:
            if recognition.group_id in approved:
                new_recognitions.append(recognition)
        return new_recognitions


class PositionFilter(Filter):

    def __init__(self):
        self.center = rospy.get_param("/openpose/net_output_width", 2) / 2

    def filter(self, recognitions):
        best_score = 9999
        best_id = -1
        new_recognitions = []
        for recognition in recognitions:
            if recognition.categorical_distribution.probabilities[0].label == "Neck":
                distance = abs(self.center - recognition.roi.x_offset)
                if distance < best_score:
                    best_score = distance
                    best_id = recognition.group_id
        for recognition in recognitions:
            if recognition.group_id == best_id:
                new_recognitions.append(recognition)
        return new_recognitions
