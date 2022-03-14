from opencog.type_constructors import (
    VariableNode,
    ConceptNode,
    TypeNode,
    TypedVariableLink,
    GetLink,
    AndLink,
    PresentLink,
    VariableList,
    NotLink,
    OrLink
)

import rospy

# Note: this is how you query for the state of a specific ConceptNode
# DefineLink(
#     DefinedPredicateNode("is in state"),
#     LambdaLink(
#          VariableList(VariableNode("target"), VariableNode("state")),
#          EqualLink(SetLink(VariableNode("state")), GetLink(VariableNode("anystate"), StateLink(VariableNode("target"), VariableNode("anystate"))))))


class IntentionRecognition(object):
    def __init__(self, world_state):
        self.ws = world_state
        approaching = self.ws._approaching
        self.intentions = set()
        self.picker = VariableNode("picker")
        self.place = VariableNode("place")
        concept = TypeNode("ConceptNode")
        human = ConceptNode("human")
        self.variables1 = TypedVariableLink(self.picker, concept)
        self.variables2 = VariableList(
            TypedVariableLink(self.picker, concept),
            TypedVariableLink(self.place, concept),
        )
        # TODO: write functions that can check these things with ConceptNode input
        self.getting_crate = lambda picker: GetLink(
            self.variables1,
            AndLink(
                PresentLink(
                    *list(filter(lambda x: x is not None,
                                 [picker,
                                  self.ws.is_a(
                                      picker, human, False)[1],
                                     self.ws.has_no_crate(
                                         picker, False)[1],
                                     self.ws.is_picker_movement(
                                         picker, approaching, False)[1],
                                     self.ws.called_robot(
                                         picker, False)[1]
                                  ]))
                            ),
                self.ws.is_a(picker, human, False)[0],
                self.ws.has_no_crate(picker, False)[0],
                # self.ws.not_dismissed_robot(picker),
                OrLink(
                    self.ws.is_picker_movement(picker, approaching, False)[0],
                    self.ws.called_robot(picker, False)[0]
                )
            )
        )

        self.getting_crate_soon = lambda picker: GetLink(
            self.variables1,
            AndLink(
                PresentLink(
                    *list(filter(lambda x: x is not None,
                                 [
                                    picker,
                                    self.ws.is_a(picker, human, False)[1],
                                    self.ws.has_no_crate(picker, False)[1],
                                    self.ws.not_called_robot(picker, False)[1],
                                    self.ws.is_picker_movement(
                                        picker, approaching, False)[1]
                                  ]))
                            ),
                self.ws.is_a(picker, human, False)[0],
                self.ws.is_picker_movement(
                    picker, standing, False)[0],
                self.ws.has_no_crate(picker, False)[0],
                self.ws.not_called_robot(picker, False)[0]
            ),
        )

        self.exchanging_crate_soon = lambda picker: GetLink(
            self.variables1,
            AndLink(
                PresentLink(
                    *list(filter(lambda x: x is not None,
                                 [
                                     picker,
                                     self.ws.is_a(picker, human, False)[1],
                                     self.ws.has_crate(picker, False)[1],
                                     self.ws.not_called_robot(
                                         picker, False)[1],
                                     self.ws.is_picker_crate_full(
                                         picker, False)[1],
                                     self.ws.not_called_robot(picker, False)[1]
                                     ]))
                            ),
                self.ws.is_a(picker, human, False)[0],
                self.ws.is_picker_movement(
                    picker, standing, False)[0],
                self.ws.has_crate(picker, False)[0],
                self.ws.is_picker_crate_full(picker, False)[0],
                self.ws.not_called_robot(picker, False)[0]
            ),
        )

        self.exchanging_crate = lambda picker: GetLink(
            self.variables2,
            AndLink(
                PresentLink(
                    *list(filter(lambda x: x is not None,
                                 [
                                    picker,
                                    self.ws.is_a(
                                        picker, human, False)[1],
                                    self.ws.is_a(
                                        self.place, ConceptNode("place"), False)[1],
                                    self.ws.has_crate(
                                        picker, False)[1],
                                    self.ws.is_at(
                                        picker, self.place, False)[1],
                                    self.ws.called_robot(
                                        picker, False)[1],
                                    self.ws.is_picker_movement(
                                        picker, approaching, False)[1],
                                    self.ws.is_picker_crate_full(
                                        picker, False)[1],
                                    self.ws.is_picker_crate_fullness_unknown(
                                        picker, False)[1],
                                    self.ws.not_called_robot(
                                        picker, False)[1],
                                    self.ws.is_unlikely_to_gesture(
                                        picker, False)[1],
                                    self.ws.not_has_picker_intention(
                                        picker,
                                        self.ws._wants_to_pass, False)[1],
                                    self.ws.has_berries(
                                        self.place, False)[1]
                                 ]))
                            ),
                self.ws.is_a(picker, human, False)[0],
                self.ws.is_a(self.place, ConceptNode("place"), False)[0],
                self.ws.has_crate(picker, False)[0],
                self.ws.is_at(picker, self.place, False)[0],
                self.ws.not_has_picker_intention(
                    picker, self.ws._wants_to_pass, False)[0],
                OrLink(
                    self.ws.called_robot(picker, False)[0],
                    AndLink(
                        self.ws.is_picker_movement(
                            picker, approaching, False)[0],
                        OrLink(
                            self.ws.is_picker_crate_full(picker, False)[0],
                            AndLink(
                                self.ws.is_picker_crate_fullness_unknown(
                                    picker, False)[0],
                                OrLink(
                                    self.ws.is_unlikely_to_gesture(
                                        picker, False)[0],
                                    self.ws.has_berries(self.place, False)[0]
                                )
                            )
                        )
                    )
                )
            )
        )

        self.passing_us = lambda picker: GetLink(
            self.variables2,
            AndLink(
                PresentLink(
                    *list(filter(lambda x: x is not None,
                                 [
                                     picker,
                                     self.ws.is_a(picker, human, False)[1],
                                     self.ws.is_a(self.place,
                                                  ConceptNode("place"),
                                                  False)[1],
                                     self.ws.is_at(
                                         picker, self.place, False)[1],
                                     self.ws.has_crate(picker, False)[1],
                                     self.ws.is_picker_movement(
                                         picker, approaching, False)[1],
                                     self.ws.not_called_robot(
                                         picker, False)[1],
                                     self.ws.is_picker_crate_empty(
                                         picker, False)[1],
                                     self.ws.has_no_berries(
                                         self.place, False)[1],
                                     self.ws.is_likely_to_gesture(
                                         picker, False)[1],
                                     self.ws.is_picker_crate_fullness_unknown(picker, False)[
                                                                              1]
                                     ]))
                            ),
                self.ws.is_a(picker, human, False)[0],
                self.ws.is_a(self.place, ConceptNode("place"), False)[0],
                self.ws.is_at(picker, self.place, False)[0],
                self.ws.has_crate(picker, False)[0],
                self.ws.is_picker_movement(picker, approaching, False)[0],
                self.ws.not_called_robot(picker, False)[0],
                OrLink(
                    self.ws.is_picker_crate_empty(picker, False)[0],
                    AndLink(
                        self.ws.has_no_berries(self.place, False)[0],
                        self.ws.is_likely_to_gesture(picker, False)[0],
                        self.ws.is_picker_crate_fullness_unknown(
                            picker, False)[0]
                        )
                )
            )
        )

        self.no_help_needed = lambda picker: GetLink(
            self.variables1,
            AndLink(
                PresentLink(
                    *list(filter(lambda x: x is not None,
                                 [
                                     picker,
                                     self.ws.is_a(picker, human, False)[1],
                                     self.ws.has_crate(picker, False)[1],
                                     self.ws.is_picker_crate_empty(
                                         picker, False)[1],
                                     self.ws.not_called_robot(
                                         picker, False)[1],
                                     self.ws.is_picker_movement(
                                         picker, approaching, False)[1]
                                     ]))
                            ),
                self.ws.is_a(picker, human, False)[0],
                self.ws.has_crate(picker, False)[0],
                self.ws.is_picker_crate_empty(picker, False)[0],
                self.ws.not_called_robot(picker, False)[0],
                NotLink(self.ws.is_picker_movement(
                    picker, approaching, False)[0]),
            ),
        )

        self.human_intention_templates = [
            (self.getting_crate, self.ws._wants_to_get_crate),
            (self.getting_crate_soon, self.ws._needs_help_soon),
            (self.exchanging_crate_soon, self.ws._needs_help_soon),
            # (self.needs_help_soon, self.ws._needs_help_soon),
            # (self.exchanging_crate1, self.ws._wants_to_exchange_their_crate),
            # (self.exchanging_crate2, self.ws._wants_to_exchange_their_crate),
            (self.exchanging_crate, self.ws._wants_to_exchange_their_crate),
            (self.passing_us, self.ws._wants_to_pass),
            (self.no_help_needed, self.ws._wants_nothing),
        ]

    def run_untargeted(self):
        intentions = set()
        for query, intention in self.human_intention_templates:
            results = self.ws.kb.execute(query(self.picker))
            pickers = []
            if query in [self.passing_us, self.exchanging_crate]:
                try:
                    pickers = results.get_out()
                    for listlink in pickers:
                        picker = listlink.get_out()[0]
                        self.ws.set_picker_intention(
                            picker, intention)
                        intentions.add((picker.name, intention))
                        # if intention.__name__ == 'wants_to_get_crate':
                        #     self.ws.kb.debug=1
                        # rospy.logwarn("{:} intention: {:}".format(
                        #                 picker.name, intention))
                except IndexError:
                    pass
            else:
                try:
                    pickers = results.get_out()
                    for picker in pickers:
                        self.ws.set_picker_intention(
                            picker, intention)
                        # rospy.logwarn("{:} intention: {:}".format(
                        #                 picker.name, intention))
                        intentions.add((picker.name, intention))
                        # if intention.__name__ == 'wants_to_get_crate':
                        #     self.ws.kb.debug=1
                except IndexError:
                    pass
        if self.intentions ^ intentions:
            if intentions:
                message = "IRE: Detected human intentions:"
                for picker, intention in intentions:
                    message += "\n  {} {}".format(picker, intention.name)
                rospy.loginfo(message)
            # if not intentions:
            #     for intention_tuple in self.intentions:
            #         self.ws.unset_picker_intention(
            #             ConceptNode(intention_tuple[0]), intention_tuple[1])
            self.intentions = intentions
