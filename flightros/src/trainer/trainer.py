#!/usr/bin/env python
import rospy

class Trainer:
    EPOCH_T = 5

    def __init__(self):
        self._state_sub = None
        self._reset_pub = None
        self._geno_type_pub = None
        self._is_resetting = False
        self._population = None
        self._current_reward = 0
        self._current_agent = None

    def _reset(self):
        self._is_resetting = True
        # set reward
        self._current_agent = self._get_next_agent()
        self._current_reward = 0
        # reset simulator
        # reset controller

    def _get_next_agent(self):
        pass

    def _next_generation(self):
        pass

    def state_callback(self, msg):
        if self._is_resetting:
            return
        # update reward
        # if T > EPOCh_T
        # reset

if __name__ == '__main__':
    rospy.init_node('trainer', anonymous=True)
    t = Trainer()
    t.run()
