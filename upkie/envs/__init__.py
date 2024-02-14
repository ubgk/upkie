#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import logging

import gymnasium as gym

from .upkie_base_env import UpkieBaseEnv

__all__ = [
    "UpkieBaseEnv",
    "register",
]

__envs__ = {}

try:
    from .upkie_ground_velocity import UpkieGroundVelocity

    __all__.append("UpkieGroundVelocity")
    __envs__["UpkieGroundVelocity"] = UpkieGroundVelocity
except ImportError as import_error:
    __envs__["UpkieGroundVelocity"] = import_error

try:
    from .upkie_servos import UpkieServos

    __all__.append("UpkieServos")
    __envs__["UpkieServos"] = UpkieServos
except ImportError as import_error:
    __envs__["UpkieServos"] = import_error


def register() -> None:
    """!
    Register Upkie environments with Gymnasium.
    """
    for env_name, Env in __envs__.items():
        if isinstance(Env, ModuleNotFoundError):
            import_error = str(Env)
            logging.warning(
                f"Cannot register {env_name} "
                f"due to missing dependency: {import_error}"
            )
            logging.info(
                "To install optional dependencies: "
                "``pip install upkie[the_full_monty]``"
            )
        else:  # valid gym.Env subclass
            gym.envs.registration.register(
                id=f"{env_name}-v{Env.version}",
                entry_point=f"upkie.envs:{env_name}",
            )
