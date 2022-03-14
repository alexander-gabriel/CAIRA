#!/usr/bin/env python
import itertools
from hashlib import sha256
import json
import os
from time import time
from datetime import timedelta

import rospy
import rosnode
import subprocess
# import roslib.scriptutil as scriptutil
from experiment import Experiment
from common.parameters import CONFIG_DIRECTORY, STATE_DIRECTORY

class Experimenter:

    def __init__(self):
        rospy.loginfo("EXP: Initializing Experimenter")
        self.last_clock = 0
        self.configs = []
        self.current_config = None

    def generate_id(self, thing):
        return sha256(json.dumps(thing, sort_keys=True)).hexdigest()

    def run(self):
        try:
            with open(os.path.join(CONFIG_DIRECTORY, STATE_DIRECTORY, "experiments.json"),
                      "r") as file:
                self.state = json.load(file)
        except Exception as err:
            self.state = {
                "finished experiments": []
            }
        number_of_runs = 0
        count = 0
        duration = 0
        durations = []
        average_duration = 0
        for config in self.configs:
            if config["run_id"] not in self.state["finished experiments"]:
                number_of_runs +=1
        remaining_runs = number_of_runs
        for config in self.configs:
            id = config["run_id"]
            if id not in self.state["finished experiments"]:
                rospy.logwarn(("EXPE: {} of {} runs finished, {} remain.\n"
                               "      Took {}s, Average: {}s, Remaining: {}")
                              .format(
                                count, number_of_runs, remaining_runs,
                                duration, average_duration,
                                timedelta(seconds=remaining_runs * average_duration)))
                running_nodes = rosnode.get_node_names()
                wait_count = 0
                if ("/thorvald_001/picker_mover" in running_nodes
                    or '/thorvald_001/scheduler' in running_nodes
                    or '/thorvald_001/action_recognition_node' in running_nodes):
                    rospy.logwarn("EXPE: Waiting for previous experiment to stop")
                    rospy.logwarn(running_nodes)
                while ("/thorvald_001/picker_mover" in running_nodes
                    or '/thorvald_001/scheduler' in running_nodes
                    or '/thorvald_001/action_recognition_node' in running_nodes):
                    if wait_count == 10:
                        process = subprocess.Popen(["/opt/ros/melodic/bin/rosnode", "cleanup", "/thorvald_001/scheduler"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
                        process.stdin.write("y\n")
                        rospy.loginfo(process.communicate()[0])
                        process.stdin.close()
                        # pinged, unpinged = rosnode.rosnode_ping_all()
                        # if unpinged:
                        #     master = scriptutil.get_master()
                        #     rospy.logwarn(master)
                        #     rospy.loginfo("Unable to contact the following nodes:")
                        #     rospy.loginfo('\n'.join(' * %s'%n for n in unpinged))
                        #     rospy.loginfo("cleanup will purge all information about these nodes from the master")
                        #     rosnode.cleanup_master_blacklist(master, unpinged)
                    elif wait_count >= 10:
                        rospy.logerr("EXPE: Zombie nodes exist")
                    rospy.sleep(1)
                    running_nodes = rosnode.get_node_names()
                    wait_count += 1
                rospy.logwarn("EXPE: Starting Experiment run {}".format(id))
                start = time()
                experiment = Experiment(config)
                experiment.setup()
                experiment.spin()
                rospy.loginfo("EXPE: Ending Experiment run {}".format(id))
                self.state["finished experiments"].append(id)
                with open(os.path.join(CONFIG_DIRECTORY,
                                       STATE_DIRECTORY,
                                       "experiments.json"),
                          "w") as file:
                    json.dump(self.state, file, sort_keys=True, indent=4,
                              separators=(',', ': '))
                duration = time() - start
                old_count = count
                count += 1
                remaining_runs -= 1
                try:
                    average_duration = (average_duration*old_count + duration)/count
                except ZeroDivisionError:
                    average_duration = duration




        # for config in self.configs:
        #     self.current_config = config
        #     for parameters in config:
        #         experiment = Experiment(parameters, config)
        #         experiment.setup()
        #         experiment.spin()
        #         # config.reset()

    def _generate_set(self, config):
        keys, values = zip(*sorted(config.items()))
        return [dict(zip(keys, v)) for v in itertools.product(*values)]
        # try:
        #     return self.parameter_set
        # except AttributeError:
        #     try:
        #         keys, values = zip(*sorted(self.parameters.items()))
        #         self.parameter_set = [dict(zip(keys, v))
        #                               for v in itertools.product(*values)]
        #         for parameters in self.parameter_set:
        #             parameters["experiment_id"] = sha256(
        #                 json.dumps(parameters, sort_keys=True)
        #             ).hexdigest()
        #     except ValueError:
        #         self.parameter_set = [{}]
        #     return self.parameter_set

    def _create_experiment(self, run, parameters, ):
        return Experiment()

    def add_experiment_set(self, experiment_set):
        common = experiment_set["common"]
        parameter_set = self._generate_set(experiment_set["parameters"])
        run_set = self._generate_set(experiment_set["runs"])
        for parameters in parameter_set:
            experiment_id = self.generate_id(parameters)
            for run in run_set:
                config = {"experiment_id": experiment_id}
                config.update(common)
                config.update(parameters)
                config.update(run)
                run_id = self.generate_id(config)
                config["run_id"] = run_id
                self.configs.append(config)

    # def add_config(self, config):
    #     self.configs.append(config)
