from threading import RLock
import time

# from opencog.atomspace import types
from opencog.ure import BackwardChainer
from opencog.scheme_wrapper import scheme_eval
from opencog.bindlink import execute_atom, evaluate_atom
from opencog.logger import log
from opencog.type_constructors import (
    PredicateNode,
    ConceptNode,
    VariableNode,
    TypedVariableLink,
    TruthValue,
    StateLink,
    ListLink,
    AndLink,
    NotLink,
    InheritanceLink,
    IdenticalLink,
    NumberNode,
    ValueOfLink,
    PlusLink,
    GetLink,
    EqualLink,
    EvaluationLink,
    VariableList,
    TypeNode,
    MinusLink,
    DivideLink,
    AccumulateLink,
    TimesLink,
    MemberLink,
    DefinedSchemaNode,
    ExecutionLink,
    SchemaNode,
    BindLink,
    ExecutionOutputLink,
    GroundedSchemaNode,
    DefineLink,
    DefinedPredicateNode,
    PredicateFormulaLink,
    LambdaLink,
    ImplicationLink
)
import rospy

from common.utils import atomspace
from common.parameters import ITERATIONS, COMPLEXITY_PENALTY


class KnowledgeBase(object):

    def __init__(self):
        rospy.loginfo("KNB: Initializing Knowledge Base")
        self.atomspace = atomspace
        # initialize_opencog(self.atomspace)
        self.debug = 0
        self.lock = RLock()
        self.TRUE = TruthValue(1, 1)
        self.FALSE = TruthValue(0, 1)
        rbs = self.build_deduction_rulebase()
        self.build_linked_deduction(rbs)
        self.build_ontology()
        self.build_defines()
        # self.build_inheritance_deduction(rbs)
        rospy.loginfo("KNB: Initialization finished")

    def reason(self, query, variables):
        rospy.logdebug("KNB: Query:\n{:}\nVariables:\n{}".format(str(query),
                                                                 variables))
        start_time = time.time()
        with self.lock:
            if self.debug == 1:
                log.use_stdout()
                log.set_level("DEBUG")
            chainer = BackwardChainer(
                    _as=atomspace, rbs=ConceptNode("deduction-rule-base"),
                    trace_as=None, control_as=None, focus_set=None,
                    target=query, vardecl=variables)
            chainer.do_chain()
            log.set_level("ERROR")
        rospy.logdebug("KNB: do_chain -- {:.4f}".format(
                                                    time.time() - start_time))
        results = chainer.get_results()
        if self.debug == 1:
            self.debug_reasoning(query, results, variables)
        return results

    def debug_reasoning(self, query, results, variables):
        rospy.loginfo("KNB: Query:\n{:}\nVariables:\n{}".format(str(query),
                                                                variables))
        rospy.loginfo(results)
        got_one = False
        if query.type_name == "GetLink":
            for result in results.get_out()[0].get_out():
                got_one = True
                # rospy.loginfo("WST: Chaining Result:{:}".format(result))
                # rospy.loginfo("WST: ---------------------")
        else:
            for result in results.get_out():
                # if result.tv == self.TRUE:
                got_one = True
                if result.tv != self.TRUE:
                    rospy.logwarn("Untrue Results:\n{}".format(results))
                # rospy.loginfo("WST: Chaining Result:{:}".format(result))
                # rospy.loginfo("WST: ---------------------")
        if not got_one:
            self.debug += 1
            if query.type_name == "GetLink":
                query = query.get_out()[1]
            rospy.logwarn("Query:\n{:}\n\n\nResults:------------\n"
                          .format(query))
            if query.type_name == "StateLink" \
                    or query.type_name == "EvaluationLink" \
                    or query.type_name == "InheritanceLink":
                with self.lock:
                    # rospy.loginfo("WST: -----------")
                    # rospy.loginfo("WST: Condition: {:}".format(query))
                    chainer = BackwardChainer(
                            self.atomspace, ConceptNode("deduction-rule-base"),
                            query)
                    chainer.do_chain()
                    res = chainer.get_results()
                got_anoter_one = False
                # rospy.logwarn("WST: Condition Results: {:}".format(res))
                for result in res.get_out():
                    # rospy.loginfo("WST: Condition Result: {:}".format(result))
                    # rospy.loginfo("WST: Condition truth: {:}"
                    #               .format(result.tv))
                    got_anoter_one = True
                    if result.tv != self.TRUE:
                        rospy.logwarn("KNB: Non-True truth value: \n{:}\n{:} -- {}"
                                      .format(query, result, result.tv))
                if not got_anoter_one:
                    rospy.logwarn("KNB: No results for: {:}".format(query))
            else:
                # rospy.loginfo("default supported query? {}".format(query))
                for condition in query.get_out():
                    with self.lock:
                        rospy.loginfo("WST: -----------")
                        rospy.loginfo("WST: Condition: {:}".format(condition))
            #             chainer = BackwardChainer(
            #                     self.atomspace,
            #                     ConceptNode("deduction-rule-base"),
            #                     condition)
            #             chainer.do_chain()
            #             res = chainer.get_results()
            #         # rospy.logwarn("WST: Condition Results: {:}".format(res))
            #         got_anoter_one = False
            #         for result in res.get_out():
            #             # rospy.loginfo("WST: Condition Result: {:}"
            #             #               .format(result))
            #             # rospy.loginfo("WST: Condition truth: {:}"
            #             #               .format(result.tv))
            #             got_anoter_one = True
            #             if result.tv != self.TRUE:
            #                 rospy.logwarn("KNB: NonTrue truth value: {:}\n{:} -- {}"
            #                               .format(condition, result, result.tv))
            #         if not got_anoter_one:
            #             rospy.logwarn("KNB: No results for: {:}"
            #                           .format(condition))
            #             if condition.type_name == "VariableList":
            #                 raise Exception()

    def execute(self, query):
        with self.lock:
            # rospy.logdebug("check variables: {}".format(query.get_out()[0]))
            # rospy.logdebug("check query: {}".format(query.get_out()[1]))
            results = execute_atom(self.atomspace, query)
            # rospy.logdebug("check results: {}".format(results))
        # for result in results.get_out():
        #     if result.tv == self.TRUE:
        #         rospy.logdebug("WST: Execution Result Truth: {:}"
        #                        .format(result.tv))
        #         rospy.logdebug("WST: Execution Result:{:}".format(result))
        #         rospy.logdebug("WST: ---------------------")
        return results

    def evaluate(self, query):
        with self.lock:
            results = evaluate_atom(self.atomspace, query)
        # for result in results.get_out():
        #     if result.tv == self.TRUE:
        #         rospy.logdebug("WST: Evalutation Result Truth: {:}"
        #                        .format(result.tv))
        #         rospy.logdebug("WST: Evalutation Result:{:}".format(result))
        #         rospy.logdebug("WST: ---------------------")
        return results

    # likely doesn't work
    # def recursive_query_matcher(self, query, result, target):
    #     rospy.loginfo("---------\nquery:\n{}\nresult:\n{}".format(query, result))
    #     if query == result:
    #         return True
    #     elif query.is_link():
    #         subnodes_results = []
    #         query_out = query.get_out()
    #         result_out = result.get_out()
    #         if query.is_a(types.OrderedLink):
    #             for index in range(len(query_out)):
    #                 subnodes_results.append(self.recursive_query_matcher(query_out[index], result_out[index], target))
    #             if False in subnodes_results:
    #                 return False
    #             else:
    #                 return subnodes_results
    #         else:
    #             equality_counter = 0
    #             different_subqueries = []
    #             relevant_subresults = copy(result_out)
    #             for subquery in query_out:
    #                 if subquery in result_out:
    #                     relevant_subresults.remove(subquery)
    #                     equality_counter += 1
    #                 else:
    #                     different_subqueries.append(subquery)
    #             for subquery in different_subqueries:
    #                 candidates = []
    #                 for subresult in relevant_subresults:
    #                     if subresult.type_name == subquery.type_name:
    #                         subnodes_results.append(self.recursive_query_matcher(subquery, subresult, target))
    #                 filtered_subnode_results = []
    #                 for subnode_result in subnodes_results:
    #                     if subnode_result:
    #
    #     elif query.type_name == "VariableNode":
    #         return (query.name, result.name)
    #     else:
    #         return False

    def get_target(self, query, results, target):
        for result in results.get_out():
            if result.tv == self.TRUE:
                self.recursive_query_matcher(query, result, target)
        return target

    def build_deduction_rulebase(self):
        with self.lock:
            rbs = ConceptNode("deduction-rule-base")
            execute_code = \
                '''
            (use-modules (opencog))
            (use-modules (opencog logger) (opencog ure) (opencog exec) (opencog python))
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/term/inheritance-direct-introduction.scm") ; loading error
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/term/crisp-deduction.scm") ; loading error

            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/instantiation.scm")
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/crisp/propositional/true-conjunction-introduction.scm")
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/propositional/fuzzy-disjunction-introduction.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/propositional/fuzzy-conjunction-introduction.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/propositional/modus-ponens.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/term/deduction.scm")
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/precise-modus-ponens.scm")
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/inheritance-to-member.scm")
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/member-to-evaluation.scm")
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/member-to-inheritance.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/negation-introduction.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/not-simplification.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/not-elimination.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/wip/implication-instantiation.scm")
            ;(load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/rules/predicate/conditional-direct-evaluation.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/meta-rules/predicate/conditional-total-instantiation.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/meta-rules/predicate/conditional-partial-instantiation.scm")
            (load-from-path "/home/4D6/einalex/Projekte/PhD/Software/pln/opencog/pln/meta-rules/predicate/universal-total-instantiation.scm")
            (define rbs (Concept "deduction-rule-base"))
            (ure-set-complexity-penalty rbs {:f})
            '''.format(COMPLEXITY_PENALTY)
            scheme_eval(self.atomspace, execute_code)
            # MemberLink(DefinedSchemaNode("inheritance-to-member-rule"), rbs)
            # MemberLink(DefinedSchemaNode("member-to-evaluation-0-rule"), rbs)
            # MemberLink(DefinedSchemaNode("member-to-evaluation-1-rule"), rbs)
            # MemberLink(DefinedSchemaNode("member-to-evaluation-2-1-rule"), rbs)
            # MemberLink(DefinedSchemaNode("member-to-evaluation-2-2-rule"), rbs)
            # MemberLink(DefinedSchemaNode("member-to-inheritance-rule"), rbs)
            # MemberLink(DefinedSchemaNode("true-conjunction-introduction-1ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("true-conjunction-introduction-2ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("true-conjunction-introduction-3ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("true-conjunction-introduction-4ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("true-conjunction-introduction-5ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-1ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-2ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-3ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-4ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-5ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-6ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-7ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-8ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-9ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-10ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-11ary-rule"), rbs)
            MemberLink(DefinedSchemaNode(
                "fuzzy-conjunction-introduction-12ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-1ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-2ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-3ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-4ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-5ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-6ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-7ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-8ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-9ary-rule"), rbs)
            # MemberLink(DefinedSchemaNode("bc-deduction-rule"), rbs)

            # MemberLink(DefinedSchemaNode("inheritance-direct-introduction-rule"), rbs)
            # MemberLink(DefinedSchemaNode("conditional-direct-evaluation-implication-scope-rule"), rbs)
            # MemberLink(DefinedSchemaNode("conditional-full-instantiation-implication-meta-rule"), rbs)
            # MemberLink(DefinedSchemaNode("conditional-full-instantiation-inheritance-meta-rule"), rbs)
            # MemberLink(DefinedSchemaNode("conditional-partial-instantiation-meta-rule"), rbs)
            # MemberLink(DefinedSchemaNode("universal-full-instantiation-forall-1ary-meta-rule"), rbs)
            # MemberLink(DefinedSchemaNode("deduction-inheritance-rule"), rbs)
            # MemberLink(DefinedSchemaNode("deduction-implication-rule"), rbs)
            # MemberLink(DefinedSchemaNode("deduction-subset-rule"), rbs)
            # MemberLink(DefinedSchemaNode("present-deduction-inheritance-rule"), rbs)
            # MemberLink(DefinedSchemaNode("modus-ponens-inheritance-rule"), rbs)
            # MemberLink(DefinedSchemaNode("modus-ponens-implication-rule"), rbs)
            # MemberLink(DefinedSchemaNode("modus-ponens-subset-rule"), rbs)
            # MemberLink(DefinedSchemaNode("crisp-contraposition-implication-scope-rule"), rbs)
            # MemberLink(DefinedSchemaNode("contraposition-implication-rule"), rbs)
            # MemberLink(DefinedSchemaNode("contraposition-inheritance-rule"), rbs)
            # MemberLink(DefinedSchemaNode("implication-scope-to-implication-rule"), rbs)
            # MemberLink(DefinedSchemaNode("implication-full-instantiation-rule"), rbs)
            # MemberLink(DefinedSchemaNode("implication-partial-instantiation-rule"), rbs)
            MemberLink(DefinedSchemaNode("negation-introduction-rule"), rbs)
            # MemberLink(DefinedSchemaNode("not-simplification-rule"), rbs)
            # MemberLink(DefinedSchemaNode("not-elimination-rule"), rbs)
            EvaluationLink(PredicateNode("URE:attention-allocation"),
                           rbs).tv = TruthValue(0.1, 1)
            ExecutionLink(SchemaNode("URE:maximum-iterations"),
                          rbs, NumberNode(ITERATIONS))
            # self.build_linked_deduction(rbs)
        return rbs

    def build_inheritance_deduction(self, deduction_rbs):
        deduction_rule = BindLink(
            VariableList(
                TypedVariableLink(
                    VariableNode('$A'),
                    TypeNode('ConceptNode')),
                TypedVariableLink(
                    VariableNode('$B'),
                    TypeNode('ConceptNode')),
                TypedVariableLink(
                    VariableNode('$C'),
                    TypeNode('ConceptNode'))),
            AndLink(
                InheritanceLink(
                    VariableNode('$A'),
                    VariableNode('$B')),
                InheritanceLink(
                    VariableNode('$B'),
                    VariableNode('$C')),
                NotLink(
                    EqualLink(
                        VariableNode('$A'),
                        VariableNode('$C')))),
            ExecutionOutputLink(
                GroundedSchemaNode('py: deduction_formula'),
                ListLink(
                    InheritanceLink(
                        VariableNode('$A'),
                        VariableNode('$C')),
                    InheritanceLink(
                        VariableNode('$A'),
                        VariableNode('$B')),
                    InheritanceLink(
                        VariableNode('$B'),
                        VariableNode('$C')))))
        deduction_rule_name = DefinedSchemaNode("inheritance-deduction-rule")
        DefineLink(
            deduction_rule_name,
            deduction_rule)
        MemberLink(deduction_rule_name, deduction_rbs)

    def build_implication_deduction(self, deduction_rbs):
        deduction_rule = BindLink(
            VariableList(
                TypedVariableLink(
                    VariableNode('$A'),
                    TypeNode('ConceptNode')),
                TypedVariableLink(
                    VariableNode('$B'),
                    TypeNode('ConceptNode')),
                TypedVariableLink(
                    VariableNode('$C'),
                    TypeNode('ConceptNode'))),
            AndLink(
                ImplicationLink(
                    VariableNode('$A'),
                    VariableNode('$B')),
                ImplicationLink(
                    VariableNode('$B'),
                    VariableNode('$C')),
                NotLink(
                    EqualLink(
                        VariableNode('$A'),
                        VariableNode('$C')))),
            ExecutionOutputLink(
                GroundedSchemaNode('py: deduction_formula'),
                ListLink(
                    ImplicationLink(
                        VariableNode('$A'),
                        VariableNode('$C')),
                    ImplicationLink(
                        VariableNode('$A'),
                        VariableNode('$B')),
                    ImplicationLink(
                        VariableNode('$B'),
                        VariableNode('$C')))))
        deduction_rule_name = DefinedSchemaNode("implication-deduction-rule")
        DefineLink(
            deduction_rule_name,
            deduction_rule)
        MemberLink(deduction_rule_name, deduction_rbs)

    def build_linked_deduction(self, deduction_rbs):
        deduction_rule = BindLink(
            VariableList(
                TypedVariableLink(
                    VariableNode('$A'),
                    TypeNode('ConceptNode')),
                TypedVariableLink(
                    VariableNode('$B'),
                    TypeNode('ConceptNode')),
                TypedVariableLink(
                    VariableNode('$C'),
                    TypeNode('ConceptNode'))),
            AndLink(
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(
                        VariableNode('$A'),
                        VariableNode('$B'))),
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(
                        VariableNode('$B'),
                        VariableNode('$C'))),
                NotLink(
                    IdenticalLink(
                        VariableNode('$A'),
                        VariableNode('$C')))),
            ExecutionOutputLink(
                GroundedSchemaNode('py: deduction_formula'),
                ListLink(
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(
                            VariableNode('$A'),
                            VariableNode('$C'))),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(
                            VariableNode('$A'),
                            VariableNode('$B'))),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(
                            VariableNode('$B'),
                            VariableNode('$C'))))))
        deduction_rule_name = DefinedSchemaNode("linked-deduction-rule")
        DefineLink(
            deduction_rule_name,
            deduction_rule)
        MemberLink(deduction_rule_name, deduction_rbs)

    def build_ontology(self):
        thing = ConceptNode("thing")
        thing.tv = self.TRUE

        concept = ConceptNode("concept")
        concept.tv = self.TRUE
        InheritanceLink(concept, thing).tv = self.TRUE

        self.place = ConceptNode("place")
        self.place.tv = self.TRUE
        InheritanceLink(self.place, concept).tv = self.TRUE

        entity = ConceptNode("entity")
        entity.tv = self.TRUE
        InheritanceLink(entity, thing).tv = self.TRUE

        obj = ConceptNode("object")
        obj.tv = self.TRUE
        organism = ConceptNode("organism")
        organism.tv = self.TRUE
        InheritanceLink(organism, entity).tv = self.TRUE
        InheritanceLink(obj, entity).tv = self.TRUE

        plant = ConceptNode("plant")
        plant.tv = self.TRUE
        InheritanceLink(plant, organism).tv = self.TRUE

        strawberryplant = ConceptNode("strawberryplant")
        strawberryplant.tv = self.TRUE
        InheritanceLink(strawberryplant, plant).tv = self.TRUE

        creature = ConceptNode("creature")
        creature.tv = self.TRUE
        self.robot = ConceptNode("robot")
        self.robot.tv = self.TRUE
        self.human = ConceptNode("human")
        self.human.tv = self.TRUE
        InheritanceLink(creature, organism).tv = self.TRUE
        InheritanceLink(self.robot, creature).tv = self.TRUE
        InheritanceLink(self.human, creature).tv = self.TRUE

        self.crate = ConceptNode("crate")
        self.crate.tv = self.TRUE
        InheritanceLink(self.crate, obj).tv = self.TRUE

    def build_defines(self):
        DefineLink(
            DefinedPredicateNode("is in state"),
            LambdaLink(
                VariableList(
                    VariableNode('node'),
                    VariableNode('state')
                ),
                MemberLink(
                    VariableNode('node'),
                    GetLink(
                        VariableNode('x'),
                        StateLink(
                            VariableNode('x'),
                            VariableNode('state')
                        )
                    )
                )
            )
        )

        DefineLink(
            DefinedPredicateNode("gesture likelihood"),
            PredicateFormulaLink(
                DivideLink(
                    AccumulateLink(
                        ValueOfLink(
                            VariableNode('picker'),
                            PredicateNode("gesture history")
                        )
                    ),
                    AccumulateLink(
                        PlusLink(
                            ValueOfLink(
                                VariableNode('picker'),
                                PredicateNode("gesture history")
                            ),
                            TimesLink(
                                NumberNode("-1"),
                                MinusLink(
                                    ValueOfLink(
                                        VariableNode('picker'),
                                        PredicateNode("gesture history")
                                    ),
                                    NumberNode("1")
                                )
                            )
                        )
                    )
                ),
                NumberNode('1')
            )
        )
