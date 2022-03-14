from os.path import join
from math import sqrt
# import traceback

import rospy
import rospkg

from opencog.type_constructors import (
    PredicateNode,
    ConceptNode,
    VariableNode,
    TypedVariableLink,
    FloatValue,
    AbsentLink,
    PresentLink,
    StateLink,
    ListLink,
    AndLink,
    NotLink,
    InheritanceLink,
    LinkValue,
    NumberNode,
    GreaterThanLink,
    ValueOfLink,
    SetValueLink,
    PlusLink,
    GetLink,
    EqualLink,
    EvaluationLink,
    VariableList,
    TypeNode,
    MinusLink,
    SetTVLink,
    PredicateFormulaLink,
    IsTrueLink,
    IsFalseLink,
    StrengthOfLink,
    DefinedPredicateNode,
    IdenticalLink
)

from common.parameters import CRATE_CAPACITY, \
    TIMEOUT_LENGTH, NO_BERRY_PLACES, PICKER_DISTANCE_PREFERENCE, \
    BEHAVIOR_PERCEPTION, GESTURE_PERCEPTION, SIMPLE_MODE, TARGET_PICKER


rospack = rospkg.RosPack()
path = join(rospack.get_path("rasberry_hri"), "src", "bdi")


class WorldState(object):

    _size = PredicateNode("size")
    _intention = PredicateNode("intention")
    _position = PredicateNode("position")
    _berry_count = PredicateNode("berry count")
    _has_crate = PredicateNode("has crate")
    _crate_level = PredicateNode("crate level")
    _crate_full = PredicateNode("crate is full")
    _full_crate_count = PredicateNode("full crate count")
    _empty_crate_count = PredicateNode("empty crate count")
    _distance = PredicateNode("distance")
    _gestures = PredicateNode("gesture history")
    _gesture_likelihood = PredicateNode("gesture likelihood")
    _dismissed = PredicateNode("dismissed robot")
    _called = PredicateNode("called robot")
    _movement = PredicateNode("movement")
    _seen_picking = PredicateNode("seen_picking")
    _timeout = PredicateNode("timeout")
    _approaching = ConceptNode("approaching")
    _leaving = ConceptNode("leaving")
    _standing = ConceptNode("standing")
    _location = PredicateNode("location")

    _needs_help_soon = ConceptNode("needs help soon")
    _wants_to_get_crate = ConceptNode("wants to get a crate")
    _wants_to_exchange_their_crate = ConceptNode(
        "wants to exchange their crate")
    _wants_to_pass = ConceptNode("wants to pass")
    _wants_nothing = ConceptNode("doesn't need help")

    def __init__(self, kb, me):
        self.CALLED_ROBOT = "CALLED_ROBOT"
        self.kb = kb
        self.me = me
        self.robot_direction = self._standing
        self.moving = False
        self.too_close = False

    # variance experiment
    # def is_target(self, place):
    #     return self.state2(ConceptNode("target"), place)
    # variance experiment

    # def old_state(self, concept, predicate, truth="TRUE"):
    #     statelink = StateLink(ListLink(concept, predicate), ConceptNode(truth))
    #     return statelink

    # def old_state2(self, concept, predicate):
    #     return StateLink(concept, predicate)

    def set_size(self, thing, width, length, python_mode=True):
        if python_mode:
            thing.set_value(self._size, FloatValue([width, length]))
        else:
            return SetValueLink(
                thing,
                self._size,
                FloatValue([width, length])
            )

    def get_size(self, thing, python_mode=True):
        if python_mode:
            return thing.get_value(self._size).to_list()
        else:
            return [ValueOfLink(
                thing,
                self._size
            ), None]

    def set_position(self, entity, x, y, date, python_mode=True):
        if python_mode:
            try:
                positions = self.get_position(entity)
                positions.append(FloatValue([x, y, date]))
            except Exception:
                positions = [FloatValue([x, y, date])]
            try:
                position = LinkValue(positions[-10:])
            except Exception():
                position = LinkValue(positions)
            entity.set_value(self._position, position)
        else:
            raise NotImplementedError

    def get_optimum_distance(self, person, python_mode=True):
        if python_mode:
            try:
                distances = person.get_value(self._distance).to_list()
            except AttributeError:
                distances = [PICKER_DISTANCE_PREFERENCE]
            return sum(distances) / len(distances)
        else:
            raise NotImplementedError

    def set_latest_distance(self, person, distance, python_mode=True):
        if python_mode:
            try:
                distances = person.get_value(self._distance).to_list()
            except AttributeError:
                distances = []
            distances.append(distance)
            person.set_value(self._distance, FloatValue(distances))
        else:
            raise NotImplementedError

    def update_position(self, person, place, python_mode=True):
        if python_mode:
            self.set_location(person, place)
            if place.name in NO_BERRY_PLACES:
                if person.name == TARGET_PICKER or person.name == self.me:
                    rospy.logwarn("WST: Observation: {:} is at {:} (no berries)"
                                  .format(person.name, place.name))
                else:
                    rospy.loginfo("WST: Observation: {:} is at {:} (no berries)"
                                  .format(person.name, place.name))
            else:
                if person.name == TARGET_PICKER or person.name == self.me:
                    rospy.logwarn("WST: Observation: {:} is at {:} (has berries)"
                                  .format(person.name, place.name))
                else:
                    rospy.loginfo("WST: Observation: {:} is at {:} (has berries)"
                                  .format(person.name, place.name))
        else:
            raise NotImplementedError

    def get_position(self, entity, python_mode=True):  # maybe fix to list in actions
        if python_mode:
            return entity.get_value(self._position).to_list()
        else:
            return [ValueOfLink(
                entity,
                self._position
            ), None]

    def get_distance(self, thing1, thing2, only_x=False, python_mode=True):
        if python_mode:
            p1x, p1y, _ = self.get_position(thing1)[-1].to_list()
            p2x, p2y, _ = self.get_position(thing2)[-1].to_list()
            dx = p1x - p2x
            dy = p1y - p2y
            dxs = dx * dx
            dys = dy * dy
            if only_x:
                try:
                    w1, l1 = self.get_size(thing1)
                    w2, l2 = self.get_size(thing2)
                    return abs(dx) - 0.5 * (l1 + l2)
                except AttributeError:
                    return abs(dx)
            else:
                try:
                    w1, l1 = self.get_size(thing1)
                    w2, l2 = self.get_size(thing2)
                    if dxs > dys:
                        return sqrt(dxs + dys) - 0.5 * (l1 + l2)
                    else:
                        return sqrt(dxs + dys) - 0.5 * (w1 + w2)
                except AttributeError:
                    return sqrt(dxs + dys)
        else:
            raise NotImplementedError

    def set_berry_state(self, place, ripe, python_mode=True):
        if python_mode:
            place.set_value(self._berry_count, NumberNode(str(ripe)))
        else:
            return SetValueLink(
                place,
                self._berry_count,
                NumberNode(str(ripe))
            )

    def get_berry_state(self, place, python_mode=True):
        if python_mode:
            return place.get_value(self._berry_count)
        else:
            return [ValueOfLink(
                place,
                self._berry_count
            ), None]

    def update_berry_state(self, place, ripe, python_mode=True):
        if python_mode:
            old_ripe = self.get_berry_state(place)
            self.set_berry_state(place, old_ripe + ripe)
        else:
            return self.set_berry_state(
                place,
                PlusLink(
                    self.get_berry_state(place, python_mode)[0],
                    NumberNode(str(ripe))
                ),
                python_mode
            )

    def has_berries(self, place, python_mode=True):
        if python_mode:
            return place.get_value(self._berry_count) > 0
        else:
            return [GreaterThanLink(
                ValueOfLink(
                    place,
                    self._berry_count
                ),
                NumberNode("0")
            ), None]

    def has_no_berries(self, place, python_mode=True):
        if python_mode:
            return place.get_value(self._berry_count) == 0
        else:
            return [EqualLink(
                ValueOfLink(
                    place,
                    self._berry_count
                ),
                NumberNode("0")
            ), None]

    def has_crate_formula(self, picker):
        return EvaluationLink(
            self._has_crate,
            picker
        )

    def set_crate_possession(self, picker, truth, python_mode=True):
        if python_mode:
            self.has_crate_formula(picker).tv = truth
        else:
            return SetTVLink(
                self.has_crate_formula(picker),
                truth
            )

    def has_crate(self, picker, python_mode=True):
        if python_mode:
            return self.has_crate_formula(picker).tv == self.kb.TRUE
        else:
            return [
                IsTrueLink(self.has_crate_formula(picker)),
                self.has_crate_formula(picker)
            ]

    def has_no_crate(self, picker, python_mode=True):
        if python_mode:
            return self.has_crate_formula(picker).tv == self.kb.FALSE
        else:
            return [
                IsFalseLink(self.has_crate_formula(picker)),
                self.has_crate_formula(picker)
            ]

    def gesture_likelihood_formula(self, person):
        return EvaluationLink(
            self._gesture_likelihood,
            person
        )

    def attach_gesture_likelihood(self, person, python_mode=True):
        if python_mode:
            self.kb.execute(SetTVLink(
                    self.gesture_likelihood_formula(person),
                    DefinedPredicateNode('gesture likelihood'),
                    ListLink(person)
                )
            )
        else:
            return SetTVLink(
                    self.gesture_likelihood_formula(person),
                    DefinedPredicateNode('gesture likelihood'),
                    ListLink(person)
                )

    def is_likely_to_gesture(self, person, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [GreaterThanLink(
                StrengthOfLink(self.gesture_likelihood_formula(
                    person)),
                NumberNode("0.5")
            ), self.gesture_likelihood_formula(person)]
        # try:
        #     gesture_history = person.get_value(self._gestures).to_list()
        # except AttributeError:
        #     gesture_history = [0.75]
        # return (sum(gesture_history) / len(gesture_history)) > 0.5

    def is_unlikely_to_gesture(self, person, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [GreaterThanLink(
                        NumberNode("0.5"),
                        StrengthOfLink(
                            self.gesture_likelihood_formula(person)
                        )
                    ), self.gesture_likelihood_formula(person)]

    def add_gesture_event(self, person, did_they_gesture, python_mode=True):
        if python_mode:
            try:
                gesture_history = person.get_value(self._gestures).to_list()
            except AttributeError:
                gesture_history = []
            if did_they_gesture:
                gesture_history.append(1.0)
            else:
                gesture_history.append(0.0)
            person.set_value(self._gestures, FloatValue(gesture_history))
        else:
            raise NotImplementedError

    def robot_has_crate(self, robot, crate_type, python_mode=True):
        if python_mode:
            return robot.get_value(crate_type) > 0
        else:
            return [GreaterThanLink(
                ValueOfLink(
                    robot,
                    crate_type
                ),
                NumberNode("0")
            ), None]

    def robot_has_no_crate(self, robot, crate_type, python_mode=True):
        if python_mode:
            return robot.get_value(crate_type) == 0
        else:
            return [EqualLink(
                ValueOfLink(
                    robot,
                    crate_type
                ),
                NumberNode("0")
            ), None]

    def robot_has_crate_capacity(self, robot, crate_type, python_mode=True):
        if python_mode:
            return robot.get_value(crate_type) < CRATE_CAPACITY
        else:
            return [GreaterThanLink(
                NumberNode(str(CRATE_CAPACITY)),
                ValueOfLink(
                    robot,
                    crate_type
                )
            ), None]

    def robot_has_no_crate_capacity(self, robot, crate_type, python_mode=True):
        if python_mode:
            return robot.get_value(crate_type) == CRATE_CAPACITY
        else:
            return [EqualLink(
                ValueOfLink(
                    robot,
                    crate_type
                ),
                NumberNode(str(CRATE_CAPACITY))
            ), None]

    def robot_set_crate_count(self, robot, crate_type, crate_count, python_mode=True):
        if python_mode:
            robot.set_value(crate_type, NumberNode(str(crate_count)))
            return crate_count
        else:
            return SetValueLink(
                robot,
                crate_type,
                NumberNode(str(crate_count)),
            )

    def robot_add_crate(self, robot, crate_type, python_mode=True):
        if python_mode:
            value = float(robot.get_value(crate_type).name)
            robot.set_value(crate_type, NumberNode(str(value+1)))
            return value+1
        else:
            return SetValueLink(
                robot,
                crate_type,
                PlusLink(
                    NumberNode("1"),
                    ValueOfLink(
                        robot,
                        crate_type
                    )
                ),
            )

    def robot_remove_crate(self, robot, crate_type, python_mode=True):
        if python_mode:
            value = float(robot.get_value(crate_type).name)
            robot.set_value(crate_type, NumberNode(str(value-1)))
            return value-1
        else:
            return SetValueLink(
                robot,
                crate_type,
                MinusLink(
                    ValueOfLink(robot, crate_type),
                    NumberNode("1")
                ),
            )

    def robot_get_crate_count(self, robot, crate_type, python_mode=True):
        if python_mode:
            try:
                return float(robot.get_value(crate_type).name)
            except AttributeError as err:
                rospy.logerr(
                    "WS: robot_get_crate_count AttributeError {}".format(err))
                return "unknown"
            except TypeError as err:
                rospy.logerr(
                    "WS: robot_get_crate_count TypeError {}".format(err))
                rospy.logwarn("WS: {}".format(robot.get_value(crate_type)))
                return "unknown"
        else:
            return [ValueOfLink(
                robot,
                crate_type
            ), None]

    def set_picker_crate_level(self, picker, level, python_mode=True):
        if python_mode:
            picker.set_value(self._crate_level, NumberNode(str(level)))
        else:
            return SetValueLink(
                picker,
                self._crate_level,
                NumberNode(str(level))
            )

    def get_picker_crate_level(self, picker, python_mode=True):
        if python_mode:
            return picker.get_value(self._crate_level)
        else:
            return [ValueOfLink(
                picker,
                self._crate_level
            ), None]

    def is_picker_crate_full(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [GreaterThanLink(
                self.get_picker_crate_level(picker, python_mode)[0],
                NumberNode('0.66')
            ), self.get_picker_crate_level(picker, python_mode)[1]]

    def is_picker_crate_fullness_unknown(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [AndLink(
                GreaterThanLink(
                    NumberNode('0.66'),
                    self.get_picker_crate_level(picker, python_mode)[0]
                ),
                GreaterThanLink(
                    self.get_picker_crate_level(picker, python_mode)[0],
                    NumberNode('0.33')
                )
            ), self.get_picker_crate_level(picker, python_mode)[1]]

    def is_picker_crate_empty(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [GreaterThanLink(
                NumberNode('0.33'),
                self.get_picker_crate_level(picker, python_mode)[0]
            ), self.get_picker_crate_level(picker, python_mode)[1]]

    def timeout_reset(self, picker, python_mode=True):
        if python_mode:
            picker.set_value(self._timeout, NumberNode("-1"))
        else:
            raise NotImplementedError

    def timeout_counter_increase(self, picker, python_mode=True):
        if python_mode:
            timeout = float(picker.get_value(self._timeout).name) + 1
            picker.set_value(self._timeout, NumberNode(str(timeout)))
        else:
            return SetValueLink(
                picker,
                self._timeout,
                PlusLink(
                    ValueOfLink(
                        picker,
                        self._timeout
                    ),
                    NumberNode('1')
                )
            )

    def timeout_not_reached(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [GreaterThanLink(
                NumberNode(str(TIMEOUT_LENGTH)),
                ValueOfLink(
                    picker,
                    self._timeout
                )
            ), None]

    def picker_intention_formula(self, picker, intention):
        return EvaluationLink(
            self._intention,
            ListLink(
                picker,
                intention
            )
        )

    def set_picker_intention(self, picker, intention, python_mode=True):
        if python_mode:
            self.picker_intention_formula(picker, intention).tv = self.kb.TRUE
        else:
            return SetTVLink(
                self.picker_intention_formula(picker, intention),
                PredicateFormulaLink(NumberNode('1'), NumberNode('1'))
            )

    def unset_picker_intention(self, picker, intention, python_mode=True):
        if python_mode:
            self.picker_intention_formula(picker, intention).tv = self.kb.FALSE
        else:
            return SetTVLink(
                self.picker_intention_formula(picker, intention),
                PredicateFormulaLink(NumberNode('0'), NumberNode('0'))
            )

    def has_picker_intention(self, picker, intention, python_mode=True):
        if python_mode:
            return self.picker_intention_formula(picker, intention)
        else:
            return [
                IsTrueLink(self.picker_intention_formula(picker, intention)),
                self.picker_intention_formula(picker, intention)
            ]

    def not_has_picker_intention(self, picker, intention, python_mode=True):
        if python_mode:
            return self.picker_intention_formula(picker, intention)
        else:
            return [
                IsFalseLink(self.picker_intention_formula(picker, intention)),
                self.picker_intention_formula(picker, intention)
            ]
    # def wants_something(self, picker, python_mode=True):
    #     if python_mode:
    #         return EvaluationLink(
    #             self._intention,
    #             ListLink(
    #                 picker,
    #                 ConceptNode("something")
    #             )
    #         )
    #     else:
    #         return SetTVLink(
    #             EvaluationLink(
    #                 self._intention,
    #                 ListLink(
    #                     picker,
    #                     ConceptNode("something")
    #                 )
    #             ),
    #             PredicateFormulaLink(NumberNode('1'), NumberNode('1'))
    #         )
    #
    # def wants_nothing(self, picker, python_mode=True):
    #     if python_mode:
    #         return EvaluationLink(
    #             self._intention,
    #             ListLink(
    #                 picker,
    #                 ConceptNode("nothing")
    #             )
    #         )
    #     else:
    #         return SetTVLink(
    #             EvaluationLink(
    #                 self._intention,
    #                 ListLink(
    #                     picker,
    #                     ConceptNode("nothing")
    #                 )
    #             ),
    #             PredicateFormulaLink(NumberNode('1'), NumberNode('1'))
    #         )
    #
    # def wants_to_pass(self, picker, python_mode=True):
    #     if python_mode:
    #         return EvaluationLink(
    #             self._intention,
    #             ListLink(
    #                 picker,
    #                 ConceptNode("wants to pass")
    #             )
    #         )
    #     else:
    #         return SetTVLink(
    #             EvaluationLink(
    #                 self._intention,
    #                 ListLink(
    #                     picker,
    #                     ConceptNode("wants to pass")
    #                 )
    #             ),
    #             PredicateFormulaLink(NumberNode('1'), NumberNode('1'))
    #         )
    #
    # def needs_help_soon(self, picker, python_mode=True):
    #     if python_mode:
    #         return EvaluationLink(
    #             self._intention,
    #             ListLink(
    #                 picker,
    #                 ConceptNode("needs help soon")
    #             )
    #         )
    #     else:
    #         return SetTVLink(
    #             EvaluationLink(
    #                 self._intention,
    #                 ListLink(
    #                     picker,
    #                     ConceptNode("needs help soon")
    #                 )
    #             ),
    #             PredicateFormulaLink(NumberNode('1'), NumberNode('1'))
    #         )
    #
    # def wants_to_exchange_their_crate(self, picker, python_mode=True):
    #     if python_mode:
    #         return EvaluationLink(
    #             self._intention,
    #             ListLink(
    #                 picker,
    #                 ConceptNode("wants to exchange their crate")
    #             )
    #         )
    #     else:
    #         return SetTVLink(
    #             EvaluationLink(
    #                 self._intention,
    #                 ListLink(
    #                     picker,
    #                     ConceptNode("wants to exchange their crate")
    #                 )
    #             ),
    #             PredicateFormulaLink(NumberNode('1'), NumberNode('1'))
    #         )
    #
    # def wants_to_get_crate(self, picker, python_mode=True):
    #     if python_mode:
    #         return EvaluationLink(
    #             self._intention,
    #             ListLink(
    #                 picker,
    #                 ConceptNode("wants to get a crate")
    #             )
    #         )
    #     else:
    #         return SetTVLink(
    #             EvaluationLink(
    #                 self._intention,
    #                 ListLink(
    #                     picker,
    #                     ConceptNode("wants to get a crate")
    #                 )
    #             ),
    #             PredicateFormulaLink(NumberNode('1'), NumberNode('1'))
    #         )

    def get_full_crate_count(self, robot, crate_count, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [ValueOfLink(
                robot,
                self._crate_count
            ), None]

    def set_full_crate_count(self, robot, crate_count, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return SetValueLink(
                robot,
                self._crate_count,
                crate_count
            )

    def set_location(self, thing, place, python_mode=True):
        if python_mode:
            StateLink(
                ListLink(thing, self._location),
                place
            )
        else:
            StateLink(
                ListLink(thing, self._location),
                place
            )

    # def set_location(self, thing, place, python_mode=True):
    #     if python_mode:
    #         thing.set_value(self._location, place)
    #     else:
    #         return SetValueLink(
    #             thing,
    #             self._location,
    #             place
    #         )

    # def get_location(self, thing, python_mode=True):
    #     if python_mode:
    #         return thing.get_value(self._location)
    #     else:
    #         return ValueOfLink(
    #             thing,
    #             self._location
    #         )

    def is_at(self, thing, place, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [PresentLink(
                StateLink(
                    ListLink(thing, self._location),
                    place
                )
            ), None]

    # def is_at(self, thing, place, python_mode=True):
    #     if python_mode:
    #         return self.get_location(thing, python_mode) == place
    #     else:
    #         return EqualLink(
    #                 self.get_location(thing, python_mode),
    #                 place
    #             )

    # def is_not_at(self, thing, place, python_mode=True):
    #     if python_mode:
    #         return self.get_location(thing, python_mode) != place
    #     else:
    #         return NotLink(
    #             EqualLink(
    #                 self.get_location(thing, python_mode),
    #                 place
    #             )
    #         )

    def is_not_at(self, thing, place, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [AbsentLink(
                StateLink(
                    ListLink(thing, self._location),
                    place
                )
            ), None]

    # def query_at(self, thing, place, python_mode=True):
    #     return self.state2(thing, place)

    # def query_not_at(self, thing, place, python_mode=True):
    #     return self.absent(self.state2(thing, place))

    # def is_occupied(self, place, python_mode=True):
    #     someone = VariableNode("someone")
    #     link = ExistsLink(
    #         someone,
    #         AndLink(
    #             OrLink(
    #                 self.is_a(someone, ConceptNode("human")),
    #                 self.is_a(someone, ConceptNode("robot")),
    #             ),
    #             self.is_at(someone, place),
    #         ),
    #     )
    #     return link

    # def is_not_occupied(self, place, python_mode=True):
    #     link = NotLink(self.is_occupied(place))
    #     return link

    def is_a(self, thing, category, python_mode=True):
        if python_mode:
            return InheritanceLink(
                thing,
                category
            )
        else:
            return [InheritanceLink(
                thing,
                category
            ), None]

    # def query_a(self, thing, category, python_mode=True):
    #     link = IdenticalLink(
    #         category, GetLink(InheritanceLink(thing, VariableNode("x")))
    #     )
    #     return link

    def same(self, thing1, thing2, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [IdenticalLink(
                thing1,
                thing2
            ), None]

    def not_same(self, thing1, thing2, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [NotLink(
                IdenticalLink(
                    thing1,
                    thing2
                )
            ), None]

    # def colocated(self, thing1, thing2, python_mode=True):
    #     link = EvaluationLink(
    #         PredicateNode("colocated"), ListLink(thing1, thing2)
    #     )
    #     return link
    def leads_to_formula(self, origin, destination):
        return EvaluationLink(
            PredicateNode("leads_to"),
            ListLink(
                origin,
                destination
            )
        )

    def leads_to(self, origin, destination, python_mode=True):
        if python_mode:
            return self.leads_to_formula(origin, destination)
        else:
            return [
                IsTrueLink(self.leads_to_formula(origin, destination)),
                self.leads_to_formula(origin, destination)
            ]

    def linked_formula(self, origin, destination):
        return EvaluationLink(
            PredicateNode("linked"),
            ListLink(
                origin,
                destination
            )
        )

    def linked(self, origin, destination, python_mode=True):
        if python_mode:
            return self.linked_formula(origin, destination)
        else:
            return [
                IsTrueLink(self.linked_formula(origin, destination)),
                self.linked_formula(origin, destination)
            ]

    def set_picker_movement(self, picker, movement, python_mode=True):
        if python_mode:
            picker.set_value(self._movement, movement)
        else:
            return SetValueLink(
                picker,
                self._movement,
                movement
            )

    def get_picker_movement(self, picker, python_mode=True):
        if python_mode:
            return picker.get_value(self._movement)
        else:
            return [ValueOfLink(
                picker,
                self._movement
            ), None]

    def is_picker_movement(self, picker, movement, python_mode=True):
        if python_mode:
            try:
                return self.get_picker_movement(picker, python_mode) == movement
            except Exception:
                return False
        else:
            return [EqualLink(
                movement,
                self.get_picker_movement(picker, python_mode)[0]
            ), self.get_picker_movement(picker, python_mode)[1]]
    #
    # def approaching(self, picker, python_mode=True):
    #     return EvaluationLink(
    #         ListLink(
    #             picker,
    #             self._movement
    #         ),
    #         self._approaching
    #     )
    #     # return self.state(picker, self._movement, self._approaching)
    #
    # def not_approaching(self, picker, python_mode=True):
    #     return NotLink(self.approaching)
    #     # return AbsentLink(self.state(picker, self._movement, self._approaching))
    #
    # def is_approaching(self, picker, python_mode=True):
    #     return self.is_movement(picker, self._approaching)
    #
    #
    # def leaving(self, picker, python_mode=True):
    #     return self.state(picker, self._movement, self._leaving)
    #
    # def not_leaving(self, picker, python_mode=True):
    #     return AbsentLink(self.state(picker, self._movement, self._leaving))
    #
    # def is_leaving(self, picker, python_mode=True):
    #     return self.is_movement(picker, self._leaving)
    #
    # def standing(self, picker, python_mode=True):
    #     return self.state(picker, self._movement, self._standing)
    #
    # def is_standing(self, picker, python_mode=True):
    #     return self.is_movement(picker, self._standing)
    #
    # def is_movement(self, picker, movement, python_mode=True):
    #     # rospy.logwarn(self.kb.execute(MeetLink(StateLink(ListLink(picker, self._movement), VariableNode("approaching")))))
    #     result = self.kb.evaluate(EqualLink(SetLink(ConceptNode(movement)), GetLink(StateLink(ListLink(picker, self._movement), VariableNode("leaving")))))
    #     return result == self.kb.TRUE

    # def not_standing(self, picker, python_mode=True):
    #     return AbsentLink(self.state(picker, self._movement, self._standing))

    # def seen_picking(self, picker, python_mode=True):
    #     return self.state(picker, self._seen_picking, "TRUE")
    #
    # def not_seen_picking(self, picker, python_mode=True):
    #     return self.state(picker, self._seen_picking, "FALSE")

    def picking_observed_formula(self, picker):
        return EvaluationLink(
            picker,
            self._seen_picking
        )

    def set_observed_picking(self, picker, truth, python_mode=True):
        if python_mode:
            self.picking_observed_formula(picker).tv = truth
        else:
            return SetValueLink(
                self.picking_observed_formula(picker),
                truth
            )

    def observed_picking(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                IsTrueLink(self.picking_observed_formula(picker)),
                self.picking_observed_formula(picker)
            ]

    def not_observed_picking(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                IsFalseLink(self.picking_observed_formula(picker)),
                self.picking_observed_formula(picker)
            ]

    def called_robot_formula(self, picker):
        return EvaluationLink(
            self._called,
            picker
        )

    def set_called_robot(self, picker, truth, python_mode=True):
        if python_mode:
            self.called_robot_formula(picker).tv = truth
        else:
            return SetValueLink(
                self.called_robot_formula(picker),
                truth
            )

    def called_robot(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                IsTrueLink(self.called_robot_formula(picker)),
                self.called_robot_formula(picker)
            ]

    def not_called_robot(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                IsFalseLink(self.called_robot_formula(picker)),
                self.called_robot_formula(picker)
            ]

    def unknown_called_robot(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                AndLink(
                    NotLink(IsTrueLink(self.called_robot_formula(picker))),
                    NotLink(IsFalseLink(self.called_robot_formula(picker)))
                ),
                self.called_robot_formula(picker)
            ]

    def dismissed_robot_formula(self, picker):
        return EvaluationLink(
            picker,
            self._dismissed
        )

    def set_dismissed_robot(self, picker, truth, python_mode=True):
        if python_mode:
            self.dismissed_robot_formula(picker).tv = truth
        else:
            return SetValueLink(
                self.dismissed_robot_formula(picker),
                truth
            )

    def dismissed_robot(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                IsTrueLink(self.dismissed_robot_formula(picker)),
                self.dismissed_robot_formula(picker)
            ]

    def not_dismissed_robot(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                IsFalseLink(self.dismissed_robot_formula(picker)),
                self.dismissed_robot_formula(picker)
            ]

    def unknown_dismissed_robot(self, picker, python_mode=True):
        if python_mode:
            raise NotImplementedError
        else:
            return [
                AndLink(
                    NotLink(IsTrueLink(self.dismissed_robot_formula(picker))),
                    NotLink(IsFalseLink(self.dismissed_robot_formula(picker)))
                ),
                self.dismissed_robot_formula(picker)
            ]

    # add a place ConceptNode to the KB
    def add_place(self, name, x, y, truth_value=None, python_mode=True):
        if python_mode:
            truth_value = self.kb.TRUE if truth_value is None else truth_value
            node1 = ConceptNode(name)
            self.set_position(node1, x, y, 0, python_mode)
            node1.tv = truth_value
            link = self.is_a(node1, self.kb.place, python_mode)
            link.tv = truth_value
            return node1
        else:
            raise NotImplementedError

    # add two 'linked' places
    def add_place_link(self, place1, place2, truth_value=None, python_mode=True):
        if python_mode:
            truth_value = self.kb.TRUE if truth_value is None else truth_value
            p1 = ConceptNode(place1)
            p1.tv = truth_value
            self.is_a(p1, self.kb.place, python_mode).tv = truth_value

            p2 = ConceptNode(place2)
            p2.tv = truth_value
            self.is_a(p2, self.kb.place, python_mode).tv = truth_value
            self.leads_to(p1, p2, python_mode).tv = truth_value
            EvaluationLink(
                PredicateNode("leads_to"), ListLink(p1, p2)
            ).tv = truth_value
            EvaluationLink(
                PredicateNode("linked"), ListLink(p1, p2)
            ).tv = truth_value
            rospy.logdebug(
                "WST: Adding place link: {:} to {:}".format(place1, place2)
            )
        else:
            raise NotImplementedError

    # add arbitrary typed things to the KB
    def add_thing(self, name, klasse, truth_value=None, python_mode=True):
        if python_mode:
            truth_value = self.kb.TRUE if truth_value is None else truth_value
            node1 = ConceptNode(name)
            node1.tv = truth_value
            node2 = ConceptNode(klasse)
            node2.tv = truth_value
            link = self.is_a(node1, node2, python_mode)
            link.tv = truth_value
            return node1
        else:
            raise NotImplementedError

    def update_action(self, person, action, error, python_mode=True):
        if python_mode:
            person = ConceptNode(person)
            if (action in ["picking berries", "picking berries right"]
                    and BEHAVIOR_PERCEPTION):
                self.set_observed_picking(person, self.kb.TRUE)
                rospy.logwarn(
                    "WST: Observation: {:} is {:} ({:.2f})".format(
                        person.name, action, error)
                )
            elif action == "calling" and (GESTURE_PERCEPTION or SIMPLE_MODE):
                self.set_called_robot(person, self.kb.TRUE)
                rospy.logwarn(
                    "WST: Observation: {:} is {:} ({:.2f})".format(
                        person.name, action, error)
                )
            elif action == "gesture cancel" and GESTURE_PERCEPTION:
                self.set_dismissed_robot(person, self.kb.TRUE)
                rospy.logwarn(
                    "WST: Observation: {:} is {:} ({:.2f})".format(
                        person.name, action, error)
                )
            elif action == "gesture stop" and GESTURE_PERCEPTION:
                rospy.logwarn(
                    "WST: Observation: {:} is {:} ({:.2f})".format(
                        person.name, action, error)
                )
                pass
                # self.set_called_robot(person, self.kb.TRUE)
            elif action == "gesture forward" and GESTURE_PERCEPTION:
                rospy.logwarn(
                    "WST: Observation: {:} is {:} ({:.2f})".format(
                        person.name, action, error)
                )
                pass
                # self.set_called_robot(person, self.kb.TRUE)
            elif action == "gesture backward" and GESTURE_PERCEPTION:
                rospy.logwarn(
                    "WST: Observation: {:} is {:} ({:.2f})".format(
                        person.name, action, error)
                )
                pass
                # self.set_called_robot(person, self.kb.TRUE)
            elif action == "neutral" or action == "handling a crate" and BEHAVIOR_PERCEPTION:
                rospy.logwarn(
                    "WST: Observation: {:} is {:} ({:.2f})".format(
                        person.name, action, error)
                )
                pass
            else:
                rospy.logerr(
                    "WST: Did not observe gesture/behaviour {}".format(action))
        else:
            raise NotImplementedError

    def get_picker_locations(self, python_mode=True):
        if python_mode:
            pickers = []
            picker = VariableNode("picker")
            location = VariableNode("location")
            variables = VariableList(
                TypedVariableLink(picker, TypeNode("ConceptNode")),
                TypedVariableLink(location, TypeNode("ConceptNode")),
            )
            query = AndLink(
                self.is_a(picker, ConceptNode("human"), False)[0],
                self.is_at(picker, location, False)[0],
            )
            results = self.kb.reason(GetLink(variables, query), variables)

            for list_link in results.get_out()[0].get_out():
                picker, location = list_link.get_out()
                pickers.append([picker, location])
            return picker
        else:
            raise NotImplementedError

    def get_location(self, target, python_mode=True):
        if python_mode:
            location = VariableNode("location")
            variables = VariableList(
                TypedVariableLink(location, TypeNode("ConceptNode"))
            )
            query = self.is_at(target, location, False)
            results = self.kb.reason(GetLink(variables, query), variables)
            for concept_node in results.get_out()[0].get_out():
                return concept_node
            return None
        else:
            raise NotImplementedError
