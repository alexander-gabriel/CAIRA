from contextlib import contextmanager
from math import pi
import os
from subprocess import Popen

import rospy

from opencog.atomspace import AtomSpace
from opencog.type_constructors import VariableNode, ConceptNode, \
                                      TypedVariableLink, TypeNode, \
                                      PredicateNode, FloatValue
from opencog.utilities import initialize_opencog

from experiments.db import DB, ARDB
from common.parameters import EXPERIMENT_ID, TARGET_PICKER, RUN_ID, EXPERIMENT_LABEL


atomspace = AtomSpace()
initialize_opencog(atomspace)


@contextmanager
def suppress(*exceptions):
    try:
        yield
    except exceptions:
        pass


class Condition(object):

    def __init__(self, label, typ):
        self.label = label
        self.typ = typ

    def get_atom(self):
        try:
            return self.typ(self.label)
        except (AttributeError, RuntimeError) as err:
            rospy.logerr(("Couldn't create Node {:}({:})"
                          "because of error: {:}").format(self.typ.__name__,
                                                          self.label, err))

    def is_variable(self):
        return self.typ == VariableNode

    def replace(self, instances):
        return self.typ(self.label)

    def __eq__(self, obj):
        if not isinstance(obj, Condition):
            return False
        else:
            return self.label == obj.label and self.typ == obj.typ


class ConceptCondition(Condition):

    def __init__(self, label):
        super(ConceptCondition, self).__init__(label, ConceptNode)

    def get_typed_variable(self):
        try:
            return TypedVariableLink(VariableNode(self.label),
                                     TypeNode(self.typ.__name__))
        except Exception as err:
            rospy.logerr(
                ("Couldn't create TypedVariableLink(VariableNode({:}), {:})"
                 "because of error: {:}").format(self.label, self.typ.__name__,
                                                 err))


class PredicateCondition(Condition):

    def __init__(self, label):
        super(PredicateCondition, self).__init__(label, PredicateNode)

    def get_typed_variable(self):
        try:
            return TypedVariableLink(VariableNode(self.label),
                                     TypeNode(self.typ.__name__))
        except Exception as err:
            rospy.logerr(
                ("Couldn't create TypedVariableLink(VariableNode({:}), {:})"
                 "because of error: {:}").format(self.label, self.typ.__name__,
                                                 err))


class NumberCondition(Condition):

    def __init__(self, label):
        super(NumberCondition, self).__init__(label, FloatValue)

    def get_typed_variable(self):
        try:
            return TypedVariableLink(VariableNode(str(self.label)),
                                     TypeNode("NumberNode"))
        except Exception as err:
            rospy.logerr(
                ("Couldn't create TypedVariableLink(VariableNode({:}), {:})"
                 "because of error: {:}").format(self.label, self.typ.__name__,
                                                 err))


class VariableCondition(Condition):

    def __init__(self, label, typ):
        super(VariableCondition, self).__init__(label, VariableNode)
        self.variable_typ = typ

    def get_typed_variable(self):
        try:
            return TypedVariableLink(VariableNode(self.label),
                                     TypeNode(self.variable_typ.__name__))
        except Exception as err:
            rospy.logerr(
                ("Couldn't create TypedVariableLink(VariableNode({:}), {:})"
                 "because of error: {:}").format(
                                self.label, self.variable_typ.__name__, err))

    def replace(self, instances):
        try:
            return self.variable_typ(instances[self.label])
        except KeyError:
            return self.typ(self.label)

    def __eq__(self, obj):
        if not isinstance(obj, VariableCondition):
            return False
        else:
            return self.label == obj.label \
                   and self.variable_typ == obj.variable_typ


class OrderedConsistentSet(object):

    def __init__(self):
        self.items = []

    def append(self, item):
        if item not in self.items:
            self.items.append(item)
            return True
        else:
            return False

    def __iadd__(self, other):
        for item in other:
            self.append(item)
        return self

    def __contains__(self, key):
        return key in self.items

    def __str__(self):
        return self.items.__str__()

    def __iter__(self):
        return self.items.__iter__()

    def __len__(self):
        return self.items.__len__()

    def __getitem__(self, key):
        return self.items.__getitem__(key)

    def __delitem__(self, key):
        return self.items.__delitem__(key)


class SortedConsistentSet(object):

    def __init__(self, reversed=False):
        self.reversed = reversed
        self.items = []

    def add(self, item):
        if item not in self.items:
            try:
                index = 0
                if self.reversed:
                    while self.items[index] > item:
                        index += 1
                else:
                    while self.items[index] < item:
                        index += 1
                self.items.insert(index, item)
            except:
                self.items.append(item)
            return True
        else:
            return False

    def __iadd__(self, other):
        for item in other:
            self.add(item)
        return self

    def __contains__(self, key):
        return key in self.items

    def __str__(self):
        return self.items.__str__()

    def __iter__(self):
        return self.items.__iter__()

    def __len__(self):
        return self.items.__len__()

    def __getitem__(self, key):
        return self.items.__getitem__(key)

    def __delitem__(self, key):
        return self.items.__delitem__(key)


def run(cmd, stdout, stderr):
    """Run a given `cmd` in a subprocess, write logs to stdout / stderr.

    Parameters
    ----------
    cmd : list of str
      Command to run.
    stdout : str or subprocess.PIPE object
      Destination of stdout output.
    stderr : str or subprocess.PIPE object
      Destination of stderr output.

    Returns
    -------
    A subprocess.Popen instance.
    """
    return Popen(cmd, stdout=stdout, stderr=stderr,
                 shell=False, preexec_fn=os.setsid)


# def start_process(cmd, typ, start_time, dpath_logs):
#     """Start a subprocess with the given command `cmd`.
#
#     Parameters
#     ----------
#     cmd : list of str
#       Command to run.
#     typ : str
#       Type of subprocess. This will be included in the logs' file names.
#     start_time : str
#       Datetime string, will be included in the logs' file names as well as
#       the resulting bag's name.
#     dpath_logs :
#       Path to log direcotry.
#
#     Returns
#     -------
#     A subprocess.Popen instance.
#     """
#     print('Starting', typ.upper())
#     stdout, stderr = get_stdout_stderr(typ, start_time, dpath_logs)
#     with open(stdout, 'wb') as out, open(stderr, 'wb') as err:
#         return run(cmd, stdout=out, stderr=err)


def wp2sym(waypoint):
    return waypoint


def combine_terms(terms):
    return zip(terms)


def sym2wp(symbol):
    return symbol


def get_angle_prototype():
    return {
        'Neck': pi,
        'Right:Wrist': pi,
        'Left:Wrist': pi,
        'Right:Elbow': pi,
        'Left:Elbow': pi,
        'Right:Shoulder': pi,
        'Left:Shoulder': pi,
        'Upper-Spine': pi,
        'Mid-Spine': pi,
        'Lower-Spine': pi,
        'Right:Hip': pi,
        'Mid:Hip': pi,
        'Left:Hip': pi,
        'Right:Knee': pi,
        'Left:Knee': pi,
        'Right:Ankle': pi,
        'Left:Ankle': pi,
        }.copy()


def get_position_prototype():
    return {
        'Neck': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Wrist': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Wrist': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Elbow': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Elbow': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Shoulder': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Shoulder': {'X': pi, 'Y': pi, 'P': 0},
        'Upper-Spine': {'X': pi, 'Y': pi, 'P': 0},
        'Mid-Spine': {'X': pi, 'Y': pi, 'P': 0},
        'Lower-Spine': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Hip': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Hip': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Knee': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Knee': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Ankle': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Ankle': {'X': pi, 'Y': pi, 'P': 0}}.copy()


def get_position_prototype_3d():
    return {
        'Neck': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Wrist': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Wrist': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Elbow': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Elbow': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Shoulder': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Shoulder': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Upper-Spine': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Mid-Spine': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Lower-Spine': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Hip': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Hip': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Knee': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Knee': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Ankle': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Ankle': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0}}.copy()


def mean(liste):
    return sum(liste)/len(liste)


db = DB(EXPERIMENT_LABEL, EXPERIMENT_ID, TARGET_PICKER, RUN_ID)
ardb = ARDB()
