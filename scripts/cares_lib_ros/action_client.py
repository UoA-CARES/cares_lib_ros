#!/usr/bin/env python3
import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus

class ActionClient(object):
    def __init__(self, action_server, action_type, feedback_type):
        self.action_server = action_server
        self.action_client = actionlib.SimpleActionClient(self.action_server, action_type)
        print(f"Waiting for {self.action_server} server")
        self.action_client.wait_for_server()

        self.status = "Idle"
        self.feedback_type = feedback_type
        self.feedback = self.feedback_type()
        self.idle = True
        print(f"ActionClient {self.action_server} ready")

    def callback_active(self):
        self.idle = False

    def callback_done(self, state, result):
        rospy.loginfo(f"{self.action_server} is done. State: {str(state)}, result: {str(result)}")
        self.idle = True
        self.feedback = self.feedback_type()
        self.status = "Done"

    def callback_feedback(self, feedback):
        self.idle = False
        self.feedback = feedback

    def send_goal(self, goal):
        self.idle = False
        self.action_client.send_goal(goal,
                                   active_cb=self.callback_active,
                                   feedback_cb=self.callback_feedback,
                                   done_cb=self.callback_done)

    def is_idle(self):
        return self.idle

    def get_state(self):
        return self.action_client.get_state(), self.feedback
    