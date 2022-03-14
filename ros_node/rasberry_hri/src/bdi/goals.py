import rospy
import time
import inspect
from copy import copy

from common.utils import db

# from opencog.bindlink import execute_atom, evaluate_atom
from opencog.atomspace import types
from opencog.type_constructors import (
    ConceptNode,
    TypedVariableLink,
    VariableList,
    TypeNode,
    AndLink,
    GetLink,
    VariableNode,
    PresentLink
)

from common.parameters import ME, DEPOT
from common.utils import OrderedConsistentSet, VariableCondition
from actions import (
    MoveAction,
    MoveToAction,
    GiveCrateAction,
    ExchangeCrateAction,
    DepositCrateAction,
    EvadeAction,
    WaitAction,
    ApproachAction,
    CloseApproachAction,
    StandByAction,
    SimpleGiveCrateAction,
    SimpleExchangeCrateAction,
    SimpleStandByAction
)
from opencog.atomspace import types


class WrongParameterException(Exception):
    def __init__(self):
        super(WrongParameterException, self).__init__()


class Goal(object):

    action_template = None
    subgoal_templates = []

    def __init__(self, world_state, robco, args, is_root=False):
        # list of subgoals (only if there is no action to be performed)
        self.is_root = is_root
        self.subgoals = []
        self.world_state = world_state
        # action to be performed (only if there are no subgoals)
        self.start_time = rospy.get_time()
        self.action = self.instantiate_action_template(
            world_state, robco, args
        )
        x, y, _ = self.world_state.get_position(ConceptNode(ME))[-1].to_list()
        if self.is_root:
            db.add_goal_entry(self.start_time, float(x), float(y),
                              self.__class__.__name__)

    @classmethod
    def instantiate_action_template(cls, world_state, robco, args):
        try:
            return cls.action_template(world_state, robco, args)
        except TypeError:
            # rospy.loginfo(
            #     "GOL: {:} initialization error: {:}".format(
            #         cls, err
            #     )
            # )
            return None

    def __eq__(self, other):
        """Override the default Equals behavior"""
        if inspect.isclass(other):
            return other.__name__ == self.__class__.__name__
        if isinstance(other, self.__class__):
            return (
                self.action_template == other.action_template
                and self.subgoal_templates == other.subgoal_templates
            )
        return False

    def __ne__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return not (
                self.action_template == other.action_template
                and self.subgoal_templates == other.subgoal_templates
            )
        return True

    def __gt__(self, other):
        return self.get_gain()/self.get_cost() > other.get_gain()/other.get_cost()

    def __lt__(self, other):
        return self.get_gain()/self.get_cost() < other.get_gain()/other.get_cost()

    def __ge__(self, other):
        return (self > other) or (self == other)

    def __le__(self, other):
        return (self < other) or (self == other)

    @classmethod
    def get_condition_templates(cls):
        try:
            cls.guide = []
            rospy.logdebug(
                "{:}.action_template.condition_templates: {}".format(
                    cls.__name__, cls.action_template.condition_templates
                )
            )
            return cls.action_template.condition_templates
        except AttributeError:
            try:
                rospy.logdebug(
                    "{:}.condition_templates: {}".format(
                        cls.__name__, cls.condition_templates
                    )
                )
                return cls.condition_templates
            except AttributeError:
                cls.condition_templates = OrderedConsistentSet()
                consequences = OrderedConsistentSet()
                for subgoal in cls.subgoal_templates:
                    new_conditions = subgoal.get_condition_templates()
                    rospy.logdebug(
                        "{:}'s subgoal {:} condition_templates: {}".format(
                            cls.__name__, subgoal.__name__, new_conditions
                        )
                    )
                    # rospy.loginfo("new conditions:")
                    # rospy.loginfo(new_conditions)
                    # rospy.loginfo("new consequences:")
                    # rospy.loginfo(consequences)
                    for condition in new_conditions:
                        if condition not in consequences:
                            cls.condition_templates.append(condition)
                    consequences += subgoal.get_consequence_templates()
                return cls.condition_templates

    @classmethod
    def get_consequence_templates(cls):
        try:
            return cls.action_template.consequence_templates
        except AttributeError:
            try:
                return cls.consequences_templates
            except AttributeError:
                cls.consequences_templates = OrderedConsistentSet()
                for subgoal in cls.subgoal_templates:
                    cls.consequences_templates += (
                        subgoal.get_consequence_templates()
                    )
                return cls.consequences_templates

    def get_conditions(self):
        try:
            return self.action.conditions
        except AttributeError:
            try:
                return self.conditions
            except AttributeError:
                self.conditions = OrderedConsistentSet()
                consequences = OrderedConsistentSet()
                for subgoal in self.subgoals:
                    new_conditions = subgoal.get_conditions()
                    for condition in new_conditions:
                        if condition not in consequences:
                            self.conditions.append(condition)
                    consequences += subgoal.get_consequences()
                return self.conditions

    def get_consequences(self):
        rospy.logdebug("GOL: Getting consequences for: {}".format(self))
        try:
            rospy.logdebug(
                "GOL: self.action.consequences: {}".format(
                    self.action.consequences
                )
            )
            return self.action.consequences
        except AttributeError:
            try:
                rospy.logdebug(
                    "GOL: self.consequences: {}".format(self.consequences)
                )
                return self.consequences
            except AttributeError:
                self.consequences = OrderedConsistentSet()
                for subgoal in self.subgoals:
                    self.consequences += subgoal.get_consequences()
                return self.consequences

    @classmethod
    def get_targets(cls, targets, variables, results):
        if results.is_a(types.ListLink):
            results_out = results.get_out()
        else:
            results_out = [results]
        for index in range(len(variables)):
            if variables[index].type_name == "TypedVariableLink":
                variable = variables[index].get_out()[0]
                # rospy.logwarn("GOL: Adding: {}: {}".format(variable.name, results_out[index].name))
                targets[variable.name] = results_out[index].name
            elif variables[index].type_name == "VariableNode":
                targets[variables[index].name] = results_out[index].name
                # rospy.logwarn("GOL: Adding: {}: {}".format(variables[index].name, results_out[index].name))
        rospy.logdebug(
            "GOL: Found targets: {:} for goal: {:}".format(
                targets, cls.__name__
            )
        )
        return targets

    @classmethod
    def build_query_fancy(
        cls, world_state, templates, picker, picker_location, me, my_location
    ):
        conditions = []
        full_conditions = []
        picker_location_var = None
        my_location_var = None
        variables = OrderedConsistentSet()
        target = {}
        for fun, args in templates:
            if fun.__name__ == "is_at" and args[0] == "picker":
                picker_location_var = args[1]
            elif fun.__name__ == "is_at" and args[0] == ME:
                my_location_var = args[1]
        for fun, args in templates:
            new_args = []
            full_args = []
            skip = False
            if fun.__name__ == "is_at" and args[0] == "picker":
                target["picker"] = picker.name
                skip = True
            elif fun.__name__ == "is_at" and args[0] == ME:
                target[ME] = me.name
                skip = True
            elif fun.__name__ == "is_a" and (args[0] == "picker"):
                target["picker"] = picker.name
                skip = True
            for arg in args:
                if arg == ME:
                    full_args.append(me)
                    if not skip:
                        new_args.append(me)
                        target[ME] = me.name
                elif arg == "picker":
                    full_args.append(picker)
                    if not skip:
                        new_args.append(picker)
                        target["picker"] = picker.name
                elif arg == picker_location_var:
                    full_args.append(picker_location)
                    if not skip:
                        new_args.append(picker_location)
                        target[picker_location_var] = picker_location.name
                elif arg == my_location_var:
                    full_args.append(my_location)
                    if not skip:
                        new_args.append(my_location)
                        target[my_location_var] = my_location.name
                elif fun.__name__ == "is_a" and args.index(arg) == 1:
                    full_args.append(ConceptNode(arg))
                    if not skip:
                        new_args.append(ConceptNode(arg))
                else:
                    variable = VariableNode(arg)
                    full_args.append(variable)
                    if not skip:
                        new_args.append(variable)
                        variables.append(
                            TypedVariableLink(
                                variable, TypeNode("ConceptNode")
                            )
                        )
            if not skip:
                new_args.append(False)
                conditions.append(fun(world_state, *new_args))
            full_args.append(False)
            full_conditions.append(fun(world_state, *full_args))
        query = AndLink(*conditions)
        # rospy.logwarn("Built query:\n{:}".format(query))
        # rospy.logwarn("Built variables:\n{:}".format(variables))
        return query, variables, target

    @classmethod
    def find_instances_fancy(cls, world_state):
        rospy.loginfo(
            "GOL: Checking goal {:} for targets --".format(cls.__name__)
        )
        targets = []
        start_time = time.time()
        templates = cls.get_condition_templates()
        picker_locations = world_state.get_picker_locations()
        me = ConceptNode(ME)
        my_location = world_state.get_location(me)
        if my_location is None:
            return targets
        for picker, picker_location in picker_locations:
            query, variables, target = cls.build_query(
                world_state,
                templates,
                picker,
                picker_location,
                me,
                my_location,
            )
            if len(variables) > 0:
                # rospy.loginfo("GOL: Reason (has args):\n{:}".format(query))
                vars = VariableList(*variables.items)
                # rospy.loginfo("GOL: Reason Variables:\n{:}".format(vars))
                results = world_state.kb.reason(query, vars)
                # rospy.logwarn("GOL: incl. variable results:\n{}".format(results))
                for result in results.get_out():
                    if result.tv == world_state.kb.TRUE:
                        rospy.loginfo(
                            "GOL: Found a target for goal: {}".format(
                                cls.__name__
                            )
                        )
                        this_target = copy(target)
                        world_state.kb.recursive_query_matcher(
                            query, result, this_target
                        )
                        # rospy.logwarn(result)
                        # rospy.logwarn(this_target)
                        targets.append(this_target)
            else:
                # rospy.loginfo("GOL: Reason (no args):\n{:}".format(query))
                results = world_state.kb.reason(query, VariableList())
                # rospy.logwarn("GOL: excl. variable results:\n{}".format(results))
                for result in results.get_out():
                    if result.tv == world_state.kb.TRUE:
                        rospy.loginfo(
                            "GOL: Found a target for goal: {}".format(
                                cls.__name__
                            )
                        )
                        this_target = copy(target)
                        # rospy.logwarn("WE'VE GOT RESULTS - NO ARGS")
                        # rospy.logwarn(result)
                        # rospy.logwarn(this_target)
                        targets.append(this_target)
        duration = time.time() - start_time
        # if duration > 0.01:
        rospy.loginfo(
            "GOL: Checked goal {:} for targets -- {:.4f}".format(
                cls.__name__, duration
            )
        )
        return targets

    @classmethod
    def build_query(cls, world_state, templates, me):
        present_conditions = []
        logic_conditions = []
        variables = OrderedConsistentSet()
        target = {}
        for fun, args in templates:
            full_args = []
            if fun.__name__ == "is_at" and args[0].label == ME:
                target[ME] = me.name
            for arg in args:
                # rospy.loginfo(fun.__name__)
                # rospy.loginfo(arg)
                full_args.append(arg.get_atom())
                if isinstance(arg, VariableCondition):
                    variables.append(arg.get_typed_variable())
            full_args.append(False)
            logic, present = fun(world_state, *full_args)
            present_conditions.append(present)
            logic_conditions.append(logic)
        query = AndLink(
            PresentLink(*list(filter(lambda x: x is not None,
                                     present_conditions))),
            *list(filter(lambda x: x is not None,
                         logic_conditions))
        )
        # rospy.logwarn("Built query:\n{:}".format(query))
        # rospy.logwarn("Built variables:\n{:}".format(variables))
        return query, variables, target

    @classmethod
    def find_instances(cls, world_state):
        # rospy.loginfo("GOL: Checking goal {:} for targets --".format(cls.__name__))
        targets = []
        # start_time = time.time()
        templates = cls.get_condition_templates()
        me = ConceptNode(ME)
        query, variables, target = cls.build_query(world_state, templates, me)
        # rospy.loginfo("GOL: Reason (has args):\n{:}".format(query))
        vars = VariableList(*variables.items)
        # rospy.loginfo("GOL: Reason Variables:\n{:}".format(vars))
        results = world_state.kb.reason(GetLink(vars, query), vars)
        # rospy.logwarn("GOL: classic reasoning results:\n{}".format(results))
        for setlink in results.get_out():
            for listlink in setlink.get_out():
                this_target = copy(target)
                # rospy.logwarn(listlink)
                cls.get_targets(this_target, variables, listlink)
                # rospy.logwarn("GOL: Found a target for goal: {}".format(cls.__name__))
                # rospy.logwarn(this_target)
                # world_state.kb.recursive_query_matcher(query, result, this_target)
                targets.append(this_target)
        # duration = time.time() - start_time
        # if duration > 0.01:
        # rospy.loginfo("GOL: Checked goal {:} for targets -- {:.4f}".format(cls.__name__, duration))
        return targets

    def is_achieved(self, world_state):
        if not self.get_action_queue():
            time = rospy.get_time()
            consequences_logic = []
            for fun, args in self.get_consequences():
                #     try:
                #         rospy.loginfo(args)
                #         args[args.index(ConceptNode(self.picker))
                #              ] = VariableNode('picker')
                #
                #     except ValueError:
                #         pass
                logic, _ = fun(world_state, *args, python_mode=False)
                if logic.type == types.PresentLink or logic.type == types.StateLink:
                    rospy.logdebug(
                        "GOL: The following consequence can not be checked:\n{:} ".format(logic))
                else:
                    consequences_logic.append(logic)
            query = AndLink(*consequences_logic)
            evaluation = world_state.kb.evaluate(query)
            if evaluation == world_state.kb.TRUE:
                if self.is_root:
                    x, y, _ = world_state.get_position(
                        ConceptNode(ME))[-1].to_list()
                    db.update_goal_entry(self.start_time, float(x), float(y),
                                         time - self.start_time)
                return True
            else:
                rospy.logwarn("GOL: {:} not achieved.".format(self))
                rospy.loginfo("GOL: Achievement test was:\n{:}".format(query))
        else:
            return False

    def get_action_queue(self):
        try:
            return self.action_queue
        except AttributeError:
            if self.action is not None:
                self.action_queue = [self.action]
            else:
                self.action_queue = []
                for subgoal in self.subgoals:
                    self.action_queue += subgoal.get_action_queue()
        return self.action_queue

    def get_next_action(self):
        try:
            return self.get_action_queue()[0]
        except IndexError:
            return None

    def perform_action(self):
        action_queue = self.get_action_queue()
        action = action_queue.pop(0)
        succeeded = action.perform()
        if not succeeded:
            # rospy.logwarn("GOL: Peforming action {:}; result: {}"
            #               .format(action, succeeded))
            action_queue.insert(0, action)
        else:
            if type(action) != WaitAction:
                rospy.loginfo(
                    "GOL: Performed action {}; Action queue is: {}".format(
                        action, action_queue
                    )
                )

    def get_cost(self):
        try:
            return self.cost
        except AttributeError:
            try:
                self.cost = self.action.get_cost()
            except AttributeError:
                self.cost = 0
                for subgoal in self.subgoals:
                    self.cost += subgoal.get_cost()
        return self.cost

    def get_gain(self):
        try:
            return self.gain
        except AttributeError:
            try:
                self.gain = self.action.gain
            except AttributeError:
                self.gain = 0
                for subgoal in self.subgoals:
                    self.gain += subgoal.get_gain()
        return self.gain


class MoveGoal(Goal):

    action_template = MoveAction

    def __init__(self, world_state, robco, args, is_root=False):
        # me, origin, destination
        super(MoveGoal, self).__init__(world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Move>"


class MoveToGoal(Goal):

    action_template = MoveToAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(MoveToGoal, self).__init__(world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Move to>"


class GiveCrateGoal(Goal):

    action_template = GiveCrateAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(GiveCrateGoal, self).__init__(world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Give crate>"


class ExchangeCrateGoal(Goal):

    action_template = ExchangeCrateAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(ExchangeCrateGoal, self).__init__(
            world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Exchange crate>"


class SimpleGiveCrateGoal(Goal):

    action_template = SimpleGiveCrateAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(SimpleGiveCrateGoal, self).__init__(
            world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Simple give crate>"


class SimpleExchangeCrateGoal(Goal):

    action_template = SimpleExchangeCrateAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(SimpleExchangeCrateGoal, self).__init__(
            world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Simple exchange crate>"


class SimpleStandByGoal(Goal):

    action_template = SimpleStandByAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(SimpleStandByGoal, self).__init__(
            world_state, robco, args, is_root)
        self.picker = args["picker"]

    def __repr__(self):
        return "<Goal:SimpleStandBy {:} ({}, {})>".format(
            self.picker, self.get_gain(), self.get_cost()
        )


class DepositCrateGoal(Goal):

    action_template = DepositCrateAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(DepositCrateGoal, self).__init__(
            world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Deposit crate>"


class EvadeGoal(Goal):

    action_template = EvadeAction

    def __init__(self, world_state, robco, args, is_root=False):
        self.picker = args["picker"]
        self.destination = args["my_destination"]
        if self.destination == args["picker_position"]:
            raise WrongParameterException()
        super(EvadeGoal, self).__init__(world_state, robco, args, is_root)

    def __repr__(self):
        return "<Goal:Evade {:} by moving to {:} ({}, {})>".format(
            self.picker, self.destination, self.get_gain(), self.get_cost()
        )


class WaitGoal(Goal):

    action_template = WaitAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(WaitGoal, self).__init__(world_state, robco, args, is_root)
        self.picker = args["picker"]

    def __repr__(self):
        return "<Goal:Wait for {:} to finish ({}, {})>".format(
            self.picker, self.get_gain(), self.get_cost()
        )


class StandByGoal(Goal):

    action_template = StandByAction

    def __init__(self, world_state, robco, args, is_root=False):
        super(StandByGoal, self).__init__(world_state, robco, args, is_root)
        self.picker = args["picker"]

    def __repr__(self):
        return "<Goal:StandBy {:} ({}, {})>".format(
            self.picker, self.get_gain(), self.get_cost()
        )


class ApproachGoal(Goal):

    action_template = ApproachAction

    def __init__(self, world_state, robco, args, is_root=False):
        d1 = world_state.get_distance(
            ConceptNode(args["my_position"]),
            ConceptNode(args["picker_position"]),
        )
        d2 = world_state.get_distance(
            ConceptNode(args["my_position"]),
            ConceptNode(args["my_destination"]),
        )
        if (d1 < d2) or (args["my_destination"] == args["picker_position"]):
            raise WrongParameterException()
        super(ApproachGoal, self).__init__(world_state, robco, args, is_root)
        self.picker = args["picker"]
        self.place = args["my_destination"]

    def __repr__(self):
        return "<Goal:Approach {:} at {:} ({}, {})>".format(
            self.picker, self.place, self.get_gain(), self.get_cost()
        )


class CloseApproachGoal(Goal):

    action_template = CloseApproachAction

    def __init__(self, world_state, robco, args, is_root=False):
        d1 = world_state.get_distance(
            ConceptNode(args["my_position"]),
            ConceptNode(args["picker_position"]),
        )
        d2 = world_state.get_distance(
            ConceptNode(args["my_position"]),
            ConceptNode(args["my_destination"]),
        )
        if (d1 < d2) or (args["my_destination"] == args["picker_position"]):
            raise WrongParameterException()
        super(CloseApproachGoal, self).__init__(
            world_state, robco, args, is_root)
        self.picker = args["picker"]
        self.place = args["my_destination"]

    def __repr__(self):
        return "<Goal:CloseApproach {:} at {:} ({}, {})>".format(
            self.picker, self.place, self.get_gain(), self.get_cost()
        )


class DeliverGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal, GiveCrateGoal]

    def __init__(self, world_state, robco, args, is_root=False):
        super(DeliverGoal, self).__init__(world_state, robco, args, is_root)
        self.subgoals.append(MoveGoal(world_state, robco, args))
        self.subgoals.append(GiveCrateGoal(world_state, robco, args))
        self.picker = args["picker"]
        self.destination = args["my_destination"]

    def __repr__(self):
        return "<Goal:Deliver crate to {:} at {:} ({}, {})>".format(
            self.picker, self.destination, self.get_gain(), self.get_cost()
        )


class ExchangeGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal, ExchangeCrateGoal]

    def __init__(self, world_state, robco, args, is_root=False):
        super(ExchangeGoal, self).__init__(world_state, robco, args, is_root)
        self.subgoals.append(MoveGoal(world_state, robco, args))
        self.subgoals.append(ExchangeCrateGoal(world_state, robco, args))
        self.picker = args["picker"]
        self.destination = args["my_destination"]

    def __repr__(self):
        return "<Goal:Exchange crate with {:} at {:} ({}, {})>".format(
            self.picker, self.destination, self.get_gain(), self.get_cost()
        )


class DepositGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal, DepositCrateGoal]

    def __init__(self, world_state, robco, args, is_root=False):
        super(DepositGoal, self).__init__(world_state, robco, args, is_root)
        self.subgoals.append(MoveGoal(world_state, robco, args))
        self.subgoals.append(DepositCrateGoal(world_state, robco, args))

    def __repr__(self):
        return "<Goal:Deposit crate at {:} ({}, {})>".format(
            DEPOT, self.get_gain(), self.get_cost()
        )


class WaitForGoal(Goal):

    action_template = None

    def __init__(self, world_state, robco, args, is_root=False):
        super(WaitForGoal, self).__init__(world_state, robco, args, is_root)
        self.subgoals.append(WaitGoal(world_state, robco, args))
        self.subgoals.append(MoveGoal(world_state, robco, args))
        self.picker = args["picker"]
        self.destination = args["my_destination"]

    def __repr__(self):
        return "<Goal:Wait for {:} to approach at {:} ({}, {})>".format(
            self.picker, self.destination, self.get_gain(), self.get_cost()
        )


class SimpleDeliverGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal, SimpleGiveCrateGoal]

    def __init__(self, world_state, robco, args, is_root=False):
        super(SimpleDeliverGoal, self).__init__(
            world_state, robco, args, is_root)
        self.subgoals.append(MoveGoal(world_state, robco, args))
        self.subgoals.append(SimpleGiveCrateGoal(world_state, robco, args))
        # self.subgoals.append(SimpleStandByGoal(world_state, robco, args))
        self.picker = args["picker"]
        self.destination = args["my_destination"]

    def __repr__(self):
        return "<Goal:SimpleDeliver crate to {:} at {:} ({}, {})>".format(
            self.picker, self.destination, self.get_gain(), self.get_cost()
        )


class SimpleExchangeGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal, SimpleExchangeCrateGoal]

    def __init__(self, world_state, robco, args, is_root=False):
        super(SimpleExchangeGoal, self).__init__(
            world_state, robco, args, is_root)
        self.subgoals.append(MoveGoal(world_state, robco, args))
        self.subgoals.append(SimpleExchangeCrateGoal(world_state, robco, args))
        # self.subgoals.append(SimpleStandByGoal(world_state, robco, args))
        self.picker = args["picker"]
        self.destination = args["my_destination"]

    def __repr__(self):
        return "<Goal:SimpleExchange crate with {:} at {:} ({}, {})>".format(
            self.picker, self.destination, self.get_gain(), self.get_cost()
        )
