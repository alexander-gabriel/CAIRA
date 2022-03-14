from time import time

import rospy

from opencog.type_constructors import (
    ConceptNode,
    TruthValue
)

from common.parameters import (
    ME,
    MOVE_GAIN,
    MEAN_WAYPOINT_DISTANCE,
    ROBOT_SPEED,
    MOVETO_GAIN,
    EVADE_GAIN,
    GIVE_GAIN,
    GIVE_COST,
    CRATE_CAPACITY,
    EXCHANGE_GAIN,
    EXCHANGE_COST,
    DEPOSIT_GAIN,
    DEPOSIT_COST,
    DEPOT,
    WAIT_GAIN,
    TIMEOUT_LENGTH,
    STANDBY_GAIN,
    APPROACH_GAIN,
    READY_POINT
)

from common.utils import VariableCondition as V
from common.utils import ConceptCondition as C
from common.utils import PredicateCondition as P
from common.utils import db
from bdi.world_state import WorldState as ws


class Action(object):

    placeholders = []
    gain = 0

    def __init__(self, world_state, args):
        self.kb = world_state.kb
        self.ws = world_state
        self.instances = args
        rospy.logdebug(
            "{:} has args: {}".format(self.__class__.__name__, args)
        )
        self.conditions = []
        self.consequences = []
        self.cost = 0
        self.first_try = True
        self.start_time = None
        replacement_table = args
        for condition in self.condition_templates:
            new_variables = list(
                map(
                    lambda variable: variable.replace(replacement_table),
                    condition[1],
                )
            )
            self.conditions.append([condition[0], new_variables])
        for consequence in self.consequence_templates:
            new_variables = list(
                map(
                    lambda variable: variable.replace(replacement_table),
                    consequence[1],
                )
            )
            self.consequences.append([consequence[0], new_variables])

    def __eq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return (
                self.instances == other.instances
                and self.conditions == other.conditions
                and self.consequences == other.consequences
            )
        return False

    def __neq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return not (
                self.instances == other.instances
                and self.conditions == other.conditions
                and self.consequences == other.consequences
            )
        return True

    def __repr__(self):
        return self.__class__.__name__

    def perform(self):
        if self.first_try:
            self.start_time = rospy.get_time()
            if type(self) != WaitAction:
                rospy.loginfo("ACT: Performing {:}".format(self))
            self.first_try = False

    def get_cost(self):
        return self.cost

    def get_gain(self):
        return self.gain


class MoveAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.not_same, [V("my_position", ConceptNode),
                       V("my_destination", ConceptNode)]],
        [ws.is_a, [V("my_destination", ConceptNode), C("place")]],
        # [ws.is_target, [V("my_destination", ConceptNode)]],  # variance experiment
        # [ws.linked, [V("my_position", ConceptNode),
        #                V("my_destination", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]]
    ]
    placeholders = [ME, "my_position", "my_destination"]

    def __init__(self, world_state, robco, args):
        super(MoveAction, self).__init__(world_state, args)
        self.robco = robco
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length = self.robco.get_path_length(
            self.position, self.destination)
        self.gain = MOVE_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(MoveAction, self).perform()
        if not self.sent_movement_request:
            # self.starting_too_close = self.ws.too_close
            # timestamp, typ, distance, x, y
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.add_action_entry(self.start_time, float(x), float(y),
                                self.__class__.__name__, "{} - {}"
                                .format(self.position, self.destination))
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
            return False
        try:
            if self.robco.get_result() or \
                (self.ws.robot_direction == self.ws._approaching
                 and self.ws.too_close):
                time = rospy.get_time()
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                self.ws.moving = False
                rospy.loginfo("ACT: Reached my destination.")
                db.update_action_entry(self.start_time, float(x), float(y),
                                       time - self.start_time)
                return True
            else:
                return False
        except AttributeError:
            return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)


class MoveToAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        # [ws.linked, [V("my_position", ConceptNode),
        #              V("my_destination", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]]
    ]
    placeholders = [ME, "picker", "my_position", "my_destination"]

    def __init__(self, world_state, robco, args):
        super(MoveToAction, self).__init__(world_state, args)
        self.robco = robco
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length = self.robco.get_path_length(self.position,
                                                      self.destination)
        self.gain = MOVETO_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(MoveToAction, self).perform()
        if self.position == self.destination:
            time = rospy.get_time()
            rospy.loginfo(
                "ACT: Reached my destination: {}".format(self.destination)
            )
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.update_action_entry(self.start_time, float(x), float(y),
                                   time - self.start_time)
            return True
        else:
            if not self.sent_movement_request:
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(x), float(y),
                                    self.__class__.__name__,
                                    "{} - {}".format(
                                    self.position,
                                    self.destination))
                self.robco.move_to(self.destination)
                self.sent_movement_request = True
                self.ws.moving = True
            try:
                result = self.robco.get_result()
                if result or self.ws.too_close:
                    self.ws.moving = False

                    rospy.loginfo(
                        "ACT: Reached my destination"
                    )
                    return True
            except Exception as err:
                rospy.logwarn("ACT: {}".format(err))
                return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:MoveTo {:}>".format(self.destination)


class EvadeAction(Action):

    condition_templates = [
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [
            ws.has_picker_intention,
            [V("picker", ConceptNode), C(ws._wants_to_pass.name)]
        ],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        ],
        [
            ws.is_not_at,
            [V("anything", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [
            ws.leads_to,
            [V("my_position", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [
            ws.leads_to,
            [V("picker_position", ConceptNode), V("my_position", ConceptNode)],
        ],
        [ws.not_same, [V("picker_position", ConceptNode),
                       V("my_destination", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]],
        [ws.not_has_picker_intention, [
            V("picker", ConceptNode), C(ws._wants_to_pass.name)]]
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        super(EvadeAction, self).__init__(world_state, args)
        self.robco = robco
        self.position = args["my_position"]
        self.picker = args["picker"]
        self.destination = args["my_destination"]
        self.path_length = self.robco.get_path_length(self.position,
                                                      self.destination)
        self.gain = EVADE_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(EvadeAction, self).perform()
        if not self.sent_movement_request:
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.add_action_entry(self.start_time, float(x), float(y),
                                self.__class__.__name__,
                                "{} - {}".format(
                                self.position,
                                self.destination))
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
            return False
        try:
            if self.robco.get_result():
                self.ws.moving = False
                time = rospy.get_time()
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.update_action_entry(self.start_time, float(x), float(y),
                                       time - self.start_time)
                self.ws.unset_picker_intention(ConceptNode(self.picker),
                                               self.ws._wants_to_pass)
                return True
            else:
                return False
        except Exception:
            return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Evade {:} by moving to {:}>".format(
            self.picker, self.destination
        )


class GiveCrateAction(Action):

    condition_templates = [
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [
            ws.has_picker_intention,
            [V("picker", ConceptNode), C(ws._wants_to_get_crate.name)]
        ],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]],
    ]
    consequence_templates = [
        [ws.not_called_robot, [V("picker", ConceptNode)]],
        [ws.not_has_picker_intention, [
            V("picker", ConceptNode), C(ws._wants_to_get_crate.name)]],
        [ws.has_crate, [V("picker", ConceptNode)]],
        [ws.dismissed_robot, [V("picker", ConceptNode)]],
        # [ws.robot_remove_crate, [C(ME), P(ws._empty_crate_count.name)]],
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        super(GiveCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = GIVE_GAIN
        self.cost = GIVE_COST

    def get_cost(self):
        return self.cost

    def perform(self):
        super(GiveCrateAction, self).perform()
        rospy.sleep(self.cost)
        picker = ConceptNode(self.picker)
        self.ws.set_called_robot(picker, self.kb.FALSE)
        self.ws.unset_picker_intention(
            picker, self.ws._wants_to_get_crate)
        self.ws.set_crate_possession(picker, self.kb.TRUE)
        self.ws.set_dismissed_robot(picker, self.kb.TRUE)
        me = ConceptNode(ME)
        empty_count = self.ws.robot_remove_crate(
            me, self.ws._empty_crate_count)
        full_count = self.ws.robot_get_crate_count(
            me, self.ws._full_crate_count)
        # for fun, args in self.consequences:
        #     new_args = []
        #     for arg in args:
        #         new_args.append(arg)
        #     consequence = fun(self.ws, *new_args)
        #     # rospy.loginfo("Entering consequence: {}".format(consequence))
        #     consequence.tv = TruthValue(1, 1)
        rospy.loginfo("ACT: {} received a crate".format(self.picker))
        rospy.loginfo("ACT: {} dismissed {}".format(self.picker, ME))
        rospy.loginfo("ACT: New robot crate state: Full/Empty {}/{}".format(
            full_count,
            empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:Give crate to {:}>".format(self.picker)


class ExchangeCrateAction(Action):

    condition_templates = [
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.robot_has_crate_capacity, [C(ME), P(ws._full_crate_count.name)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [
            ws.has_picker_intention,
            [V("picker", ConceptNode), C(ws._wants_to_exchange_their_crate.name)]
        ],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]],
    ]
    consequence_templates = [
        [ws.is_picker_crate_empty, [V("picker", ConceptNode)]],
        [ws.not_called_robot, [V("picker", ConceptNode)]],
        [ws.not_has_picker_intention, [
            V("picker", ConceptNode), C(ws._wants_to_exchange_their_crate.name)]],
        # [ws.robot_add_crate, [C(ME), P(ws._full_crate_count.name)]],
        # [ws.robot_remove_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.dismissed_robot, [V("picker", ConceptNode)]],
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        super(ExchangeCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = EXCHANGE_GAIN
        self.cost = EXCHANGE_COST

    def get_cost(self):
        return self.cost

    def perform(self):
        super(ExchangeCrateAction, self).perform()
        rospy.sleep(self.cost)
        picker = ConceptNode(self.picker)
        self.ws.set_called_robot(picker, self.kb.FALSE)
        self.ws.unset_picker_intention(
            picker, self.ws._wants_to_exchange_their_crate)
        self.ws.set_picker_crate_level(picker, 0)
        self.ws.set_dismissed_robot(picker, self.kb.TRUE)
        me = ConceptNode(ME)
        full_count = self.ws.robot_add_crate(me, self.ws._full_crate_count)
        empty_count = self.ws.robot_remove_crate(
            me, self.ws._empty_crate_count)
        # for fun, args in self.consequences:
        #     new_args = []
        #     for arg in args:
        #         new_args.append(arg)
        #     consequence = fun(self.ws, *new_args)
        #     # rospy.loginfo("Entering consequence: {}".format(consequence))
        #     consequence.tv = TruthValue(1, 1)
        rospy.loginfo("ACT: {} dismissed {}".format(self.picker, ME))

        rospy.loginfo("ACT: New robot crate state: Full/Empty {}/{}".format(
            full_count,
            empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:Exchange crate with {:}>".format(self.picker)


class DepositCrateAction(Action):

    condition_templates = [
        # [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        [ws.robot_has_no_crate_capacity, [
            C(ME), P(ws._full_crate_count.name)]],
        [ws.robot_has_no_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.same, [V("my_destination", ConceptNode), C(DEPOT)]],
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]],
    ]
    consequence_templates = [
        [
            ws.robot_has_crate_capacity,
            [C(ME), P(ws._full_crate_count.name)],
        ],
        [
            ws.robot_has_crate,
            [C(ME), P(ws._empty_crate_count.name)],
        ],
    ]
    placeholders = [ME, DEPOT, "my_destination"]

    def __init__(self, world_state, robco, args):
        super(DepositCrateAction, self).__init__(world_state, args)
        self.robco = robco
        me = ConceptNode(ME)
        self.position = args["my_destination"]
        self.gain = DEPOSIT_GAIN
        self.full_crate_count = self.ws.robot_get_crate_count(
            me, self.ws._full_crate_count)
        self.empty_crate_count = self.ws.robot_get_crate_count(
            me, self.ws._empty_crate_count)
        if self.full_crate_count == "unknown" or self.empty_crate_count == "unknown":
            raise Exception()

    def get_cost(self):
        try:
            return DEPOSIT_COST * (self.full_crate_count + CRATE_CAPACITY - self.empty_crate_count)
        except Exception:
            return DEPOSIT_COST * CRATE_CAPACITY

    def get_gain(self):
        try:
            return DEPOSIT_GAIN * (
                max(self.full_crate_count, CRATE_CAPACITY - self.empty_crate_count)
                / CRATE_CAPACITY
            )
        except Exception:
            return DEPOSIT_GAIN / 2

    def perform(self):
        super(DepositCrateAction, self).perform()
        rospy.sleep(self.get_cost())
        me = ConceptNode(ME)
        full_count = self.ws.robot_set_crate_count(
            me, self.ws._full_crate_count, 0)
        empty_count = self.ws.robot_set_crate_count(
            me, self.ws._empty_crate_count, CRATE_CAPACITY)
        # for fun, args in self.consequences:
        #     new_args = []
        #     for arg in args:
        #         new_args.append(arg)
        #     consequence = fun(self.ws, *new_args)
        #     # rospy.loginfo("Entering consequence: {}".format(consequence))
        #     consequence.tv = TruthValue(1, 1)
        rospy.loginfo(
            "ACT: New robot crate state: Full/Empty {}/{}".format(full_count, empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:Deposit crates at {:}>".format(DEPOT)


class ApproachAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [
            ws.has_picker_intention,
            [V("picker", ConceptNode), C(ws._needs_help_soon.name)]
        ],
        # [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.robot_has_crate_capacity, [C(ME), P(ws._full_crate_count.name)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)]
        ],
        [
            ws.leads_to,
            [V("my_destination", ConceptNode), V("midway_point", ConceptNode)]
        ],
        [
            ws.leads_to,
            [
                V("midway_point", ConceptNode),
                V("picker_position", ConceptNode),
            ]
        ],
        [
            ws.is_not_at,
            [V("anything", ConceptNode), V("midway_point", ConceptNode)]
        ],
        [
            ws.not_same,
            [
                V("my_destination", ConceptNode),
                V("picker_position", ConceptNode),
            ]
        ],
        [
            ws.not_same,
            [V("my_destination", ConceptNode), V("midway_point", ConceptNode)]
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("my_destination", ConceptNode)]
        ]
    ]
    consequence_templates = []
    placeholders = [
        ME,
        "my_position",
        "my_destination",
        "picker",
        "picker_position",
    ]

    def __init__(self, world_state, robco, args):
        super(ApproachAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length = self.robco.get_path_length(
            self.position, self.destination)
        self.gain = APPROACH_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(ApproachAction, self).perform()
        if self.position == self.destination:
            rospy.loginfo(
                "ACT: Reached my destination: {}".format(self.destination)
            )
            return True
        else:
            if not self.sent_movement_request:
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(x), float(y),
                                    self.__class__.__name__,
                                    "{} - {}".format(self.position,
                                                     self.destination))
                self.robco.move_to(self.destination)
                self.sent_movement_request = True
                self.ws.moving = True
            try:
                result = self.robco.get_result()
                if result:
                    time = rospy.get_time()
                    self.ws.moving = False
                    rospy.loginfo(
                        "ACT: Reached my destination: {}".format(
                            result.success
                        )
                    )
                    x, y, _ = self.ws.get_position(
                        ConceptNode(ME))[-1].to_list()
                    db.update_action_entry(self.start_time, float(x), float(y),
                                           time - self.start_time)
                    return True
            except Exception as err:
                # rospy.logwarn(err)
                return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Approach {:} at {:}>".format(
            self.picker, self.destination
        )


class CloseApproachAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [
            # doesnt exist
            ws.has_picker_intention,
            [V("picker", ConceptNode), C("something")]
        ],
        # [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        ],
        [
            ws.leads_to,
            [V("my_destination", ConceptNode),
             V("picker_position", ConceptNode)],
        ],
        [
            ws.not_same,
            [
                V("my_destination", ConceptNode),
                V("picker_position", ConceptNode),
            ],
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("my_destination", ConceptNode)],
        ],
        # [ws.is_closer, [V("my_position", ConceptNode),
        #              V("my_destination", ConceptNode),
        #              V("picker_position", ConceptNode)]]
    ]
    consequence_templates = []
    placeholders = [
        ME,
        "my_position",
        "my_destination",
        "picker",
        "picker_position",
    ]

    def __init__(self, world_state, robco, args):
        super(CloseApproachAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length = self.robco.get_path_length(
            self.position, self.destination)
        self.gain = APPROACH_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(CloseApproachAction, self).perform()
        if self.position == self.destination:
            rospy.loginfo(
                "ACT: Reached my destination: {}".format(self.destination)
            )
            return True
        else:
            if not self.sent_movement_request:
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(x), float(y),
                                    self.__class__.__name__,
                                    "{} - {}".format(self.position,
                                                     self.destination))
                self.robco.move_to(self.destination)
                self.sent_movement_request = True
                self.ws.moving = True
            try:
                # this next line will
                result = self.robco.get_result()
                if result:
                    time = rospy.get_time()
                    self.ws.moving = False
                    rospy.loginfo(
                        "ACT: Reached my destination: {}".format(
                            result.success
                        )
                    )
                    x, y, _ = self.ws.get_position(
                        ConceptNode(ME))[-1].to_list()
                    db.update_action_entry(self.start_time, float(x), float(y),
                                           time - self.start_time)
                    return True
            except Exception as err:
                # rospy.logwarn(err)
                return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:CloseApproach {:} at {:}>".format(
            self.picker, self.destination
        )


class WaitAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.unknown_called_robot, [V("picker", ConceptNode)]],
        [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        # [ws.timeout_not_reached, [V("picker", ConceptNode)]],
        [
            ws.has_picker_intention,
            [V("picker", ConceptNode), C(ws._needs_help_soon.name)]
        ],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)]
        ],
        [
            ws.leads_to,
            [V("my_position", ConceptNode), V("midway_point", ConceptNode)]
        ],
        [
            ws.leads_to,
            [
                V("midway_point", ConceptNode),
                V("picker_position", ConceptNode),
            ]
        ],
        [
            ws.is_not_at,
            [V("anything", ConceptNode), V("midway_point", ConceptNode)]
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("picker_position", ConceptNode)]
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("midway_point", ConceptNode)]
        ],
    ]
    consequence_templates = []
    placeholders = [ME, "picker", "my_position"]

    def __init__(self, world_state, robco, args):
        super(WaitAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = WAIT_GAIN
        self.timeout = -1

    def perform(self):
        super(WaitAction, self).perform()
        picker = ConceptNode(self.picker)
        current_time = time()
        if self.timeout == -1:
            self.x, self.y, _ = self.ws.get_position(
                ConceptNode(ME))[-1].to_list()
            # db.add_action_entry(self.start_time, float(x), float(y),
            #                     self.__class__.__name__,
            #                     "{}".format(self.position))
            self.timeout = current_time + TIMEOUT_LENGTH
        else:
            if current_time > self.timeout:
                self.timeout = -1
                self.ws.set_dismissed_robot(picker, self.kb.TRUE)
                rospy.loginfo("ACT: {} dismissed {}".format(self.picker, ME))
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(self.x), float(self.y),
                                    self.__class__.__name__,
                                    "{}".format(self.position))
                db.update_action_entry(self.start_time, float(x), float(y),
                                       current_time - self.start_time)
        return True

    def get_cost(self):
        return TIMEOUT_LENGTH

    def __repr__(self):
        return "<Action:Wait for {:} at {:}>".format(
            self.picker, self.position
        )


class StandByAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.dismissed_robot, [V("picker", ConceptNode)]],
        # [
        #     ws.is_at,
        #     [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        # ],
        # [
        #     ws.leads_to,
        #     [V("my_position", ConceptNode), V("picker_position", ConceptNode)],
        # ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), C(READY_POINT)],
        ],
    ]
    consequence_templates = [[ws.not_dismissed_robot,
                              [V("picker", ConceptNode)]]]
    placeholders = [ME, "picker", "my_position"]

    def __init__(self, world_state, robco, args):
        super(StandByAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.destination = READY_POINT
        self.path_length = self.robco.get_path_length(
            self.position, self.destination)
        self.gain = STANDBY_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(StandByAction, self).perform()
        if not self.sent_movement_request:
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.add_action_entry(self.start_time, float(x), float(y),
                                self.__class__.__name__, "{} - {}"
                                .format(self.position, self.destination))
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True

        try:
            result = self.robco.get_result()
            if result:
                time = rospy.get_time()
                self.ws.moving = False
                rospy.loginfo("ACT: Reached my destination: {}"
                              .format(result.success))
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.update_action_entry(self.start_time, float(x), float(y),
                                       time - self.start_time)
                picker = ConceptNode(self.picker)
                self.ws.set_dismissed_robot(picker, self.kb.FALSE)
                rospy.loginfo("ACT: removed {}'s dismissal note for {}"
                              .format(self.picker, ME))
                return True
        except Exception as err:
            rospy.logwarn(err)
            return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:StandBy {:} to {:}>".format(
            self.picker, self.destination
        )


class SimpleGiveCrateAction(Action):

    condition_templates = [
        [ws.called_robot, [V("picker", ConceptNode)]],
        [ws.has_no_crate, [V("picker", ConceptNode)]],
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
    ]
    consequence_templates = [
        [ws.has_crate, [V("picker", ConceptNode)]],
        # [ws.robot_remove_crate, [C(ME), P(ws._empty_crate_count.name)]],
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        super(SimpleGiveCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = GIVE_GAIN
        self.cost = GIVE_COST

    def get_cost(self):
        return self.cost

    def perform(self):
        super(SimpleGiveCrateAction, self).perform()
        rospy.sleep(self.cost)
        picker = ConceptNode(self.picker)
        self.ws.set_called_robot(picker, self.kb.FALSE)
        self.ws.set_crate_possession(picker, self.kb.TRUE)
        me = ConceptNode(ME)
        empty_count = self.ws.robot_remove_crate(
            me, self.ws._empty_crate_count)
        full_count = self.ws.robot_get_crate_count(
            me, self.ws._full_crate_count)
        # for fun, args in self.consequences:
        #     new_args = []
        #     for arg in args:
        #         new_args.append(arg)
        #     consequence = fun(self.ws, *new_args)
        #     # rospy.loginfo("Entering consequence: {}".format(consequence))
        #     consequence.tv = TruthValue(1, 1)
        rospy.loginfo("ACT: {} received a crate".format(self.picker))
        rospy.loginfo(
            "ACT: New robot crate state: Full/Empty {}/{}".format(full_count, empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:SimpleGiveCrate to {:}>".format(self.picker)


class SimpleExchangeCrateAction(Action):

    condition_templates = [
        [ws.called_robot, [V("picker", ConceptNode)]],
        [ws.has_crate, [V("picker", ConceptNode)]],
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.robot_has_crate_capacity, [C(ME), P(ws._full_crate_count.name)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
    ]
    consequence_templates = [
        [ws.not_called_robot, [V("picker", ConceptNode)]],
        # [ws.robot_add_crate, [C(ME), P(ws._full_crate_count.name)]],
        # [ws.robot_remove_crate, [C(ME), P(ws._empty_crate_count.name)]],
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        # [me, picker, my_destination]
        super(SimpleExchangeCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = EXCHANGE_GAIN
        self.cost = EXCHANGE_COST

    def get_cost(self):
        return self.cost

    def perform(self):
        super(SimpleExchangeCrateAction, self).perform()
        rospy.sleep(self.cost)
        picker = ConceptNode(self.picker)
        self.ws.set_called_robot(picker, self.kb.FALSE)
        me = ConceptNode(ME)
        full_count = self.ws.robot_add_crate(me, self.ws._full_crate_count)
        empty_count = self.ws.robot_remove_crate(
            me, self.ws._empty_crate_count)
        # for fun, args in self.consequences:
        #     new_args = []
        #     for arg in args:
        #         new_args.append(arg)
        #     consequence = fun(self.ws, *new_args)
        #     # rospy.loginfo("Entering consequence: {}".format(consequence))
        #     consequence.tv = TruthValue(1, 1)
        rospy.loginfo("ACT: {} dismissed {}".format(self.picker, ME))
        rospy.loginfo(
            "ACT: New robot crate state: Full/Empty {}/{}".format(full_count, empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:SimpleExchangeCrate with {:}>".format(self.picker)


class SimpleStandByAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        # [
        #     ws.is_at,
        #     [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        # ],
        # [
        #     ws.leads_to,
        #     [V("my_position", ConceptNode), V("picker_position", ConceptNode)],
        # ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), C(READY_POINT)],
        ],
    ]
    consequence_templates = []
    placeholders = [ME, "picker", "my_position"]

    def __init__(self, world_state, robco, args):
        super(SimpleStandByAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.destination = READY_POINT
        self.path_length = self.robco.get_path_length(
            self.position, self.destination)
        self.gain = STANDBY_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(SimpleStandByAction, self).perform()
        if not self.sent_movement_request:
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.add_action_entry(self.start_time, float(x), float(y),
                                self.__class__.__name__, "{} - {}"
                                .format(self.position, self.destination))
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True

        try:
            result = self.robco.get_result()
            if result:
                time = rospy.get_time()
                self.ws.moving = False
                rospy.loginfo("ACT: Reached my destination: {}"
                              .format(result.success))
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.update_action_entry(self.start_time, float(x), float(y),
                                       time - self.start_time)
                return True
        except Exception as err:
            rospy.logwarn(err)
            return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:SimpleStandBy {:} to {:}>".format(
            self.picker, self.destination
        )
