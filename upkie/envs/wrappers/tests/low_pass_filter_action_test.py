#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test NoisifyAction wrapper."""

import unittest

import gymnasium
import numpy as np

from upkie.envs.wrappers.low_pass_filter_action import LowPassFilterAction
from upkie.envs.wrappers.tests.envs import ActionObserverEnv


class LowPassFilterActionTestCase(unittest.TestCase):
    def test_lpf(self):
        env = ActionObserverEnv()
        lpf_env = LowPassFilterAction(env, time_constant=1.0)
        action = np.array([1.0])
        inner_action, _, _, _, _ = lpf_env.step(action)
        self.assertTrue(np.allclose(action * env.dt, inner_action))

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            env = gymnasium.make("Pendulum-v1")
            lpf_env = LowPassFilterAction(env, time_constant=1.0)
            check_env(lpf_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
