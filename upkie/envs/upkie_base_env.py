#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import abc
from typing import Optional, Tuple

import gymnasium
import numpy as np
from loop_rate_limiters import RateLimiter
from numpy.typing import NDArray
from vulp.spine import SpineInterface

import upkie.config
from upkie.observers.base_pitch import compute_base_pitch_from_imu
from upkie.utils.exceptions import UpkieException
from upkie.utils.nested_update import nested_update
from upkie.utils.robot_state import RobotState


class UpkieBaseEnv(abc.ABC, gymnasium.Env):

    """!
    Base class for Upkie environments.

    ### Attributes

    This base environment has the following attributes:

    - ``fall_pitch``: Fall pitch angle, in radians.
    - ``init_state``: Initial state for the floating base of the robot, which
        may be randomized upon resets.

    @note This environment is made to run on a single CPU thread rather than on
    GPU/TPU. The downside for reinforcement learning is that computations are
    not massively parallel. The upside is that it simplifies deployment to the
    real robot, as it relies on the same spine interface that runs on Upkie.
    """

    __frequency: Optional[float]
    __log: dict
    __rate: Optional[RateLimiter]
    __regulate_frequency: bool
    _spine: SpineInterface
    _spine_config: dict
    fall_pitch: float
    init_state: RobotState

    def __init__(
        self,
        fall_pitch: float = 1.0,
        frequency: Optional[float] = 200.0,
        init_state: Optional[RobotState] = None,
        regulate_frequency: bool = True,
        shm_name: str = "/vulp",
        spine_config: Optional[dict] = None,
        spine_retries: int = 10,
    ) -> None:
        """!
        Initialize environment.

        @param fall_pitch Fall detection pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz. Can be
            set even when `regulate_frequency` is false, as some environments
            make use of e.g. `self.dt` internally.
        @param init_state Initial state of the robot, only used in simulation.
        @param regulate_frequency Enables loop frequency regulation.
        @param shm_name Name of shared-memory file to exchange with the spine.
        @param spine_config Additional spine configuration overriding the
            defaults from ``//config:spine.yaml``. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
        @param spine_retries Number of times to try opening the shared-memory
            file to communicate with the spine.
        """
        merged_spine_config = upkie.config.SPINE_CONFIG.copy()
        if spine_config is not None:
            nested_update(merged_spine_config, spine_config)
        if regulate_frequency and frequency is None:
            raise UpkieException(f"{regulate_frequency=} but {frequency=}")
        if init_state is None:
            init_state = RobotState(
                position_base_in_world=np.array([0.0, 0.0, 0.6])
            )

        self.__frequency = frequency
        self.__log = {}
        self.__rate = None
        self.__regulate_frequency = regulate_frequency
        self._spine = SpineInterface(shm_name, retries=spine_retries)
        self._spine_config = merged_spine_config
        self.fall_pitch = fall_pitch
        self.init_state = init_state

    @property
    def dt(self) -> Optional[float]:
        """!
        Regulated period of the control loop in seconds, or ``None`` if there
        is no loop frequency regulation.
        """
        return 1.0 / self.__frequency if self.__frequency is not None else None

    @property
    def frequency(self) -> Optional[float]:
        """!
        Regulated frequency of the control loop in Hz, or ``None`` if there is
        no loop frequency regulation.
        """
        return self.__frequency

    def update_init_rand(self, **kwargs) -> None:
        """!
        Update initial-state randomization.

        Keyword arguments are forwarded as is to @ref
        upkie.utils.robot_state_randomization.RobotStateRandomization.update.
        """
        self.init_state.randomization.update(**kwargs)

    def close(self) -> None:
        """!
        Stop the spine properly.
        """
        self._spine.stop()

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[NDArray[float], dict]:
        """!
        Resets the spine and get an initial observation.

        @param seed Number used to initialize the environment’s internal random
            number generator.
        @param options Currently unused.
        @returns
            - ``observation``: Initial vectorized observation, i.e. an element
              of the environment's ``observation_space``.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)
        self._spine.stop()
        self.__reset_rate()
        self.__reset_init_state()
        self._spine.start(self._spine_config)
        self._spine.get_observation()  # might be a pre-reset observation
        spine_observation = self._spine.get_observation()
        self.parse_first_observation(spine_observation)
        observation = self.get_env_observation(spine_observation)
        info = {"spine_observation": spine_observation}
        return observation, info

    def __reset_rate(self):
        if self.__regulate_frequency:
            rate_name = f"{self.__class__.__name__} rate limiter"
            self.__rate = RateLimiter(self.__frequency, name=rate_name)

    def __reset_init_state(self):
        init_state, np_random = self.init_state, self.np_random
        orientation_matrix = init_state.sample_orientation(np_random)
        qx, qy, qz, qw = orientation_matrix.as_quat()
        orientation_quat = np.array([qw, qx, qy, qz])
        position = init_state.sample_position(np_random)
        linear_velocity = init_state.sample_linear_velocity(np_random)
        omega = init_state.sample_angular_velocity(np_random)

        bullet_config = self._spine_config["bullet"]
        reset = bullet_config["reset"]
        reset["orientation_base_in_world"] = orientation_quat
        reset["position_base_in_world"] = position
        reset["linear_velocity_base_to_world_in_world"] = linear_velocity
        reset["angular_velocity_base_in_base"] = omega

    def step(
        self,
        action: NDArray[float],
    ) -> Tuple[NDArray[float], float, bool, bool, dict]:
        """!
        Run one timestep of the environment's dynamics. When the end of the
        episode is reached, you are responsible for calling `reset()` to reset
        the environment's state.

        @param action Action from the agent.
        @returns
            - ``observation``: Observation of the environment, i.e. an element
              of its ``observation_space``.
            - ``reward``: Reward returned after taking the action.
            - ``terminated``: Whether the agent reached a terminal state,
              which can be a good or a bad thing. When true, the user needs to
              call :func:`reset()`.
            - ``truncated'': Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call :func:`reset()`.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              us this is the full observation dictionary coming from the spine.
        """
        if self.__regulate_frequency:
            self.__rate.sleep()  # wait until clock tick to send the action

        # Act
        spine_action = self.get_spine_action(action)
        spine_action["env"] = {}
        if self.__log:
            spine_action["env"].update(self.__log)
        if self.__regulate_frequency:
            spine_action["env"]["rate"] = {"slack": self.__rate.slack}
        self._spine.set_action(spine_action)

        # Observe
        spine_observation = self._spine.get_observation()
        observation = self.get_env_observation(spine_observation)
        reward = self.get_reward(observation, action)
        terminated = self.detect_fall(spine_observation)
        truncated = False
        info = {"spine_observation": spine_observation}
        return observation, reward, terminated, truncated, info

    def detect_fall(self, spine_observation: dict) -> bool:
        """!
        Detect a fall based on the body-to-world pitch angle.

        @param spine_observation Observation dictionary with an "imu" key.
        @returns True if and only if a fall is detected.
        """
        imu = spine_observation["imu"]
        pitch = compute_base_pitch_from_imu(imu["orientation"])
        return abs(pitch) > self.fall_pitch

    def parse_first_observation(self, spine_observation: dict) -> None:
        """!
        Parse first observation after the spine interface is initialized.

        @param spine_observation First observation.

        This method is an optional way for environments to record some state
        (abstracted away from the agnet) at reset.
        """

    @abc.abstractmethod
    def get_env_observation(self, spine_observation: dict):
        """!
        Extract environment observation from spine observation dictionary.

        @param spine_observation Spine observation dictionary.
        @returns Environment observation.
        """

    @abc.abstractmethod
    def get_reward(self, observation, action) -> float:
        """!
        Get reward from observation and action.

        @param observation Environment observation.
        @param action Environment action.
        @returns Reward.
        """

    @abc.abstractmethod
    def get_spine_action(self, action) -> dict:
        """!
        Convert environment action to a spine action dictionary.

        @param action Environment action.
        @returns Spine action dictionary.
        """

    def log(self, new_log: dict) -> None:
        """!
        Log anything to the action dictionary.

        @param new_log New log entry.
        """
        self.__log = new_log
