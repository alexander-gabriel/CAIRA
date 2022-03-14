import time
from math import sqrt

from profilehooks import profile

import rospy

from opencog.utilities import initialize_opencog
from opencog.type_constructors import (
    set_type_ctor_atomspace,
    FloatValue,
    ConceptNode,
    NumberNode
)

from std_msgs.msg import String
from std_srvs.srv import Empty
from topological_navigation.route_search import TopologicalRouteSearch

from rasberry_people_perception.topological_localiser import TopologicalNavLoc

from common.parameters import (
    NS,
    NO_BERRY_PLACES,
    ROBOT_WIDTH,
    ROBOT_LENGTH,
    PICKER_WIDTH,
    PICKER_LENGTH,
    TARGET_PICKER,
    SEEN_PICKING,
    HAS_CRATE,
    CRATE_FULL,
    MIN_GAIN,
    MAX_COST,
    FULL_CRATE_COUNT,
    EMPTY_CRATE_COUNT,
    DISMISSED_ROBOT,
    PICKERS,
    BERRY_POSITION_PERCEPTION,
    PICKER_CRATE_POSSESSION_PERCEPTION,
    PICKER_CRATE_FILL_PERCEPTION,
    ROBOT_CRATE_POSSESSION_PERCEPTION,
    ROBOT_POSITION,
    SIMPLE_MODE,
    PICKER_CRATE_FILL_STATE,
    PICKER_GESTURE_HISTORY
)

from common.utils import SortedConsistentSet
from bdi.goals import (
    ExchangeGoal,
    DeliverGoal,
    EvadeGoal,
    DepositGoal,
    WrongParameterException,
    ApproachGoal,
    CloseApproachGoal,
    StandByGoal,
    WaitForGoal,
    WaitGoal,
    MoveGoal,
    SimpleStandByGoal,
    SimpleExchangeGoal,
    SimpleDeliverGoal,
)
from bdi.robot_control import RobotControl
from bdi.world_state import WorldState
from bdi.intention_recognition import IntentionRecognition


class BDISystem:
    def __init__(self, me, kb):
        rospy.loginfo("BDI: Initializing BDI System")
        self.shutting_down = False
        self.durations = []
        self.kb = kb
        initialize_opencog(self.kb.atomspace)
        set_type_ctor_atomspace(self.kb.atomspace)
        self.last_intention = None
        self.last_distance = 0
        with self.kb.lock:
            # self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            # self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
            self.picker_pose_publisher = rospy.Publisher(
                "/picker_mover", String, queue_size=10
            )
            self.robco = RobotControl(me)
            self.world_state = WorldState(self.kb, me)
            self.intention_recognition = IntentionRecognition(self.world_state)
            rospy.loginfo("BDI: Initialized World State")
            self.last_behaviours = {}
            self.desires = []
            if SIMPLE_MODE:
                self.desires.append(WaitGoal)
                self.desires.append(DepositGoal)
                # self.desires.append(SimpleStandByGoal)
                self.desires.append(SimpleDeliverGoal)
                self.desires.append(SimpleExchangeGoal)
                # self.desires.append(DeliverGoal)
                # self.desires.append(ExchangeGoal)
            else:
                self.desires.append(WaitGoal)
                self.desires.append(DepositGoal)
                self.desires.append(StandByGoal)
                self.desires.append(DeliverGoal)
                self.desires.append(ExchangeGoal)
                self.desires.append(EvadeGoal)
                self.desires.append(ApproachGoal)
                self.desires.append(CloseApproachGoal)
            self.intentions = SortedConsistentSet(True)
            self.locator = TopologicalNavLoc()
            self.route_search = TopologicalRouteSearch(self.locator.tmap)
            rospy.loginfo("BDI: Adding Waypoints")
            target = rospy.get_param("{}/target_picker".format(NS))
            rospy.loginfo("BDI: Target picker is: {}".format(target))
            for node in self.locator.tmap.nodes:
                place = self.world_state.add_place(
                    node.name, node.pose.position.x, node.pose.position.y
                )
                if BERRY_POSITION_PERCEPTION:
                    if place.name in NO_BERRY_PLACES:
                        self.world_state.set_berry_state(place, 0)
                        rospy.loginfo("BDI: No Berries at {}"
                                      .format(place.name))
                    else:
                        self.world_state.set_berry_state(place, 1)
                # variance experiment
                # if place.name == target:
                #     self.world_state.is_target(place).tv = self.kb.TRUE
                # variance experiment
                for edge in node.edges:
                    if node.name != edge.node:
                        self.world_state.add_place_link(node.name, edge.node)
            self._setup_experiment(me)
        rospy.loginfo("BDI: Initialized BDI System")

    def _setup_experiment(self, me):
        rospy.loginfo("BDI: Adding Me: {}".format(me))
        self.me = self.world_state.add_thing(me, "robot")
        self.world_state.set_position(self.me, ROBOT_POSITION[0],
                                      ROBOT_POSITION[1], rospy.get_time())
        self.world_state.set_size(self.me, ROBOT_WIDTH, ROBOT_LENGTH)
        rospy.loginfo("BDI: Adding Pickers")
        for name in PICKERS:
            picker = self.world_state.add_thing(name, "human")
            for has_gestured in PICKER_GESTURE_HISTORY:
                self.world_state.add_gesture_event(picker, has_gestured)
            self.world_state.attach_gesture_likelihood(picker)
            self.world_state.set_picker_movement(
                picker, self.world_state._standing)
            self.world_state.set_called_robot(picker, self.kb.FALSE)
            self.world_state.unset_picker_intention(
                picker, self.world_state._wants_to_pass)
            self.world_state.unset_picker_intention(
                picker, self.world_state._wants_to_exchange_their_crate)
            self.world_state.unset_picker_intention(
                picker, self.world_state._wants_to_get_crate)
            self.world_state.unset_picker_intention(
                picker, self.world_state._wants_nothing)
            picker.set_value(self.world_state._timeout, FloatValue(-1))
            self.world_state.set_size(picker, PICKER_WIDTH, PICKER_LENGTH)
        picker = ConceptNode(TARGET_PICKER)
        rospy.loginfo("BDI: Setting Target Picker State")
        rospy.loginfo("BDI: {}'s likelihood to gesture is {:d}%.".format(
            picker.name, int(self.world_state.gesture_likelihood_formula(picker).tv.mean*100)))
        if DISMISSED_ROBOT:
            self.world_state.set_dismissed_robot(picker, self.kb.TRUE)
        else:
            self.world_state.set_dismissed_robot(picker, self.kb.FALSE)
        if SEEN_PICKING:
            self.world_state.set_observed_picking(picker, self.kb.TRUE)
        else:
            self.world_state.set_observed_picking(picker, self.kb.FALSE)
        if PICKER_CRATE_FILL_PERCEPTION:
            self.world_state.set_picker_crate_level(
                picker, PICKER_CRATE_FILL_STATE)
        if PICKER_CRATE_POSSESSION_PERCEPTION:
            if HAS_CRATE:
                rospy.loginfo("BDI: {} has a crate".format(picker.name))
                self.world_state.set_crate_possession(picker, self.kb.TRUE)
                rospy.loginfo("BDI: {}'s crate is {:d}% full.".format(
                    picker.name, int(PICKER_CRATE_FILL_STATE*100)))
            else:
                rospy.loginfo("BDI: {} has no crate".format(picker.name))
                self.world_state.set_crate_possession(picker, self.kb.FALSE)

            # if CRATE_FULL:
            #     rospy.loginfo("BDI: {}'s crate is full".format(picker.name))
            #     self.world_state.set_picker_crate_level(picker, 0.99)
            # else:
            #     rospy.loginfo("BDI: {}'s crate is empty".format(picker.name))
            #     self.world_state.set_picker_crate_level(picker, 0.01)
        if ROBOT_CRATE_POSSESSION_PERCEPTION:
            rospy.loginfo("BDI: Setting empty crate count of '{}' to {:d}."
                          .format(self.me.name, EMPTY_CRATE_COUNT))
            self.world_state.robot_set_crate_count(
                self.me,
                self.world_state._empty_crate_count,
                EMPTY_CRATE_COUNT)
            # self.me.set_value(
            #     self.world_state._empty_crate_count,
            #     NumberNode(str(EMPTY_CRATE_COUNT)))
            rospy.loginfo("BDI: Setting full crate count of '{}' to {:d}."
                          .format(self.me.name, FULL_CRATE_COUNT))
            self.world_state.robot_set_crate_count(
                self.me,
                self.world_state._full_crate_count,
                FULL_CRATE_COUNT)
            # self.me.set_value(
            #     self.world_state._full_crate_count,
            #     NumberNode(str(FULL_CRATE_COUNT))
            # )

    def _generate_intention_candidates(self):
        rospy.logdebug("BDI: Generating Behaviour Options")
        intention_candidates = SortedConsistentSet(True)
        for goal in self.desires:
            if goal not in self.intentions:
                args_list = goal.find_instances(self.world_state)
                for args in args_list:
                    if len(args) > 0:
                        try:
                            intention_candidates.add(
                                goal(self.world_state, self.robco,
                                     args, is_root=True)
                            )
                        except WrongParameterException as err:
                            pass
                        except Exception as err:
                            rospy.logerr(
                                "BDI: Couldn't add intention {:} because of error {:}".format(
                                    goal, err
                                )
                            )
        for intention in self.intentions:
            if intention in intention_candidates and intention.is_achieved(
                self.world_state
            ):
                intention_candidates.remove(intention)
        if len(intention_candidates) > 0:
            rospy.logdebug("BDI: Desires: {}".format(intention_candidates))
        return intention_candidates

    def _filter_intention_candidates(self, intention_candidates):
        rospy.logdebug("BDI: Filtering Desires")
        intentions = self.intentions or SortedConsistentSet(True)
        for candidate in intention_candidates:
            gain = candidate.get_gain()
            cost = candidate.get_cost()
            if gain >= MIN_GAIN and cost <= MAX_COST:
                intentions.add(candidate)
            else:
                rospy.logwarn(
                    (
                        "BDI: 149 - Desire {} is outside bounds "
                        "of gain >{} and cost <{}"
                    ).format(candidate, MIN_GAIN, MAX_COST)
                )
        if len(intentions) > 0:
            rospy.logdebug("BDI: Intentions: {}".format(intentions))
        return intentions

    def _perform_action(self):
        rospy.logdebug("BDI: Choosing Next Action")
        chosen_intention = None
        try:
            chosen_intention = self.intentions[0]
            if chosen_intention != self.last_intention:
                if type(chosen_intention) != WaitGoal:
                    rospy.loginfo(
                        "BDI: Following {:}".format(chosen_intention))
                    rospy.loginfo(
                        "BDI: Intention Queue: {}".format(self.intentions))
                self.last_intention = chosen_intention
            chosen_intention.perform_action()
        except IndexError:
            pass

    def loop(self):
        # self.pause()
        # rospy.logwarn("BDI: ----- Started Loop -----")
        if not self.shutting_down:
            start_time = time.time()
            # loop_start = start_time
            self._update_beliefs()
            rospy.logdebug(
                "BDI: --  updated beliefs   -- {:.4f}".format(
                    time.time() - start_time
                )
            )
            start_time = time.time()
            intention_candidates = self._generate_intention_candidates()
            rospy.logdebug(
                "BDI: -- generated desires  -- {:.4f}".format(
                    time.time() - start_time
                )
            )
            start_time = time.time()
            self.intentions = self._filter_intention_candidates(
                intention_candidates
            )
            rospy.logdebug(
                "BDI: -- filtered desires   -- {:.4f}".format(
                    time.time() - start_time
                )
            )
            start_time = time.time()
            self._perform_action()
            rospy.logdebug(
                "BDI: -- performed action   -- {:.4f}".format(
                    time.time() - start_time
                )
            )
            start_time = time.time()
            self._clean_intentions()
            rospy.logdebug(
                "BDI: -- cleaned intentions -- {:.4f}".format(
                    time.time() - start_time
                )
            )
            # duration = time.time() - loop_start
            # if duration > 0.01:
            # rospy.loginfo("BDI: -----     Loop     ----- {:.4f}"
            #               .format(duration))
            # self.unpause()
        else:
            # self.pause.close()
            # self.unpause.close()
            rospy.loginfo("BDI: Shutdown complete")

    def _clean_intentions(self):
        index = 0
        while index < len(self.intentions):
            if self.intentions[index].is_achieved(self.world_state):
                if type(self.intentions[index]) != WaitGoal:
                    rospy.loginfo(
                        "BDI: Finished following {}".format(
                            self.intentions[index])
                    )
                del self.intentions[index]
            else:
                index += 1

    def _update_beliefs(self):
        rospy.logdebug("BDI: Updating Beliefs")
        self.intention_recognition.run_untargeted()

    def add_goal(self, goal):
        """Adds a goal to the BDI system"""
        self.desires.append(goal)

    def shutdown(self):
        rospy.loginfo("BDI: Shutting down")
        self.shutting_down = True
