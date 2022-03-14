import sqlite3
import os
from contextlib import closing
import rospy
from threading import Lock

from common.parameters import CONFIG_DIRECTORY, ACTIVE_DIRECTORY, \
                              LOG_DIRECTORY, STATE_DIRECTORY


def make_paths():
    for path in [
         os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY),
         os.path.join(CONFIG_DIRECTORY, STATE_DIRECTORY),
         os.path.join(CONFIG_DIRECTORY, ACTIVE_DIRECTORY)]:
        try:
            os.makedirs(path)
        except OSError:
            if not os.path.isdir(path):
                raise


class DB:
    def __init__(self, experiment_label, experiment_id, picker_id, run_id, filename=os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, 'log.db')):
        self.run_id = run_id
        self.lock = Lock()
        make_paths()
        self.db = sqlite3.connect(filename, check_same_thread=False)
        try:
            self.build_db()
        except sqlite3.OperationalError:
            pass
        self.add_experiment(experiment_label, experiment_id, picker_id)

    def build_db(self):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    "CREATE TABLE runs (run_id text PRIMARY KEY, experiment_id text, picker_id text)")
                cursor.execute("CREATE TABLE experiments (experiment_id text PRIMARY KEY, experiment_label text, "
                               "FOREIGN KEY (experiment_id) REFERENCES runs (experiment_id))")
                cursor.execute("CREATE TABLE picker_behavior (run_id text, timestamp float, "
                               "x float, y float, orientation float, behavior text, "
                               "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
                cursor.execute("CREATE TABLE picker_waiting (run_id text, timestamp float, "
                               "x float, y float, orientation float, wait float, "
                               "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
                cursor.execute("CREATE TABLE robot_actions (run_id text, timestamp float, "
                               "duration float, start_x float, start_y float, "
                               "end_x float, end_y float, action text, info text, "
                               "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
                cursor.execute("CREATE TABLE robot_goals (run_id text, timestamp float, "
                               "duration float, start_x float, start_y float, "
                               "end_x float, end_y float, goal text, "
                               "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
                cursor.execute("CREATE TABLE meetings (run_id text, timestamp float, "
                               "x float, y float, signal_distance float, stop_distance float, speed_profile text, "
                               "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
                self.db.commit()

    # variance testing
    # def add_entry(self, timestamp, typ, target, x, y):
    #     cursor = self.db.cursor()
    #     cursor.execute("INSERT INTO movement_variance VALUES"
    #                    "(?, ?, ?, ?, ?, ?)", (self.experiment_id, timestamp, x,
    #                                           y, typ, target))
    #     self.db.commit()

    def add_person_behaviour_entry(self, timestamp, x, y,
                                   orientation, behaviour):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    "INSERT INTO picker_behavior VALUES (?, ?, ?, ?, ?, ?)",
                    (self.run_id, timestamp, x, y, orientation, behaviour))
                self.db.commit()

    def add_experiment(self, experiment_label, experiment_id, picker_id):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                try:
                    cursor.execute(
                        "INSERT INTO runs VALUES (?, ?, ?)",
                        (self.run_id, experiment_id, picker_id))
                except sqlite3.IntegrityError:
                    pass
                try:
                    cursor.execute(
                        "INSERT INTO experiments VALUES (?, ?)",
                        (experiment_id, experiment_label))
                except sqlite3.IntegrityError:
                    pass
                self.db.commit()

    def add_person_wait_entry(self, timestamp, x, y,
                              orientation, behaviour, wait):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    "INSERT INTO picker_waiting VALUES (?, ?, ?, ?, ?, ?)",
                    (self.run_id, timestamp, x, y, orientation, wait))
                self.db.commit()

    def add_meet_entry(self, distance, speed_profile):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    ("INSERT INTO meetings (run_id, signal_distance, speed_profile) "
                     "VALUES (?, ?, ?)"),
                    (self.run_id, distance, str(speed_profile)[1:-1]))
                self.db.commit()

    def update_meet_entry(self, timestamp, x, y, distance):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    ("UPDATE meetings SET timestamp = ?, x = ?, y = ?,"
                     "stop_distance = ? WHERE (run_id = ?)"),
                    (timestamp, x, y, distance, self.run_id))
                self.db.commit()
                rospy.logdebug(
                    "DBW: updated meet entry at {:.2}m distance".format(distance))

    def add_action_entry(self, timestamp, x, y, action, info):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    ("INSERT INTO robot_actions (run_id, timestamp, "
                     "start_x, start_y, action, info) VALUES (?, ?, ?, ?, ?, ?)"),
                    (self.run_id, timestamp, x, y, action, info))
                self.db.commit()

    def update_action_entry(self, timestamp, x, y, duration):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    ("UPDATE robot_actions SET end_x = ?, end_y = ?, duration = ? "
                     "WHERE (run_id = ? AND timestamp = ?)"),
                    (x, y, duration, self.run_id, timestamp))
                self.db.commit()

    def add_goal_entry(self, timestamp, x, y, goal):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    ("INSERT INTO robot_goals (run_id, timestamp, "
                     "start_x, start_y, goal) VALUES (?, ?, ?, ?, ?)"),
                    (self.run_id, timestamp, x, y, goal))
                self.db.commit()

    def update_goal_entry(self, timestamp, x, y, duration):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    ("UPDATE robot_goals SET end_x = ?, end_y = ?, duration = ? "
                     "WHERE (run_id = ? AND timestamp = ?)"),
                    (x, y, duration, self.run_id, timestamp))
                self.db.commit()

    def close_db(self):
        with self.lock:
            self.db.commit()
            self.db.close()


class ARDB:
    def __init__(self, filename=os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, 'ar-log.db')):
        self.lock = Lock()
        make_paths()
        self.db = sqlite3.connect(filename, check_same_thread=False)
        try:
            self.build_db()
        except sqlite3.OperationalError:
            pass

    def build_db(self):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    "CREATE TABLE pose_classifications (timestamp float, picker_id text, actual_behavior text, classification text)")
                self.db.commit()

    def add_classification(self, timestamp, picker, behaviour, classification):
        with self.lock:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(
                    "INSERT INTO pose_classifications VALUES (?, ?, ?, ?)",
                    (timestamp, picker, behaviour, classification))
                self.db.commit()

    def close_db(self):
        with self.lock:
            self.db.commit()
            self.db.close()
