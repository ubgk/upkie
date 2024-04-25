#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import logging
import os
import signal
import traceback
from os import path

import gin
import yaml
from loop_rate_limiters import RateLimiter
from rules_python.python.runfiles import runfiles
from servo_controller import ServoController
from vulp.spine import SpineInterface


class CompilationModeError(Exception):
    """Raised when the example is called with unexpected parameters."""


def run(
    spine: SpineInterface,
    spine_config: dict,
    frequency: float = 200.0,
) -> None:
    """!
    Read observations and send actions to the spine.

    @param spine Interface to the spine.
    @param spine_config Spine configuration dictionary.
    @param frequency Control frequency in Hz.
    """
    controller = ServoController()
    dt = 1.0 / frequency
    rate = RateLimiter(frequency, "controller")

    wheel_radius = controller.wheel_radius
    spine_config["wheel_odometry"] = {
        "signed_radius": {
            "left_wheel": +wheel_radius,
            "right_wheel": -wheel_radius,
        }
    }

    spine.start(spine_config)
    observation = spine.get_observation()  # pre-reset observation
    while True:
        observation = spine.get_observation()
        action = controller.cycle(observation, dt)
        spine.set_action(action)
        rate.sleep()


if __name__ == "__main__":
    agent_dir = path.dirname(__file__)
    deez_runfiles = runfiles.Create()
    spine_path = os.path.join(
        agent_dir,
        deez_runfiles.Rlocation("upkie/spines/bullet_spine"),
    )

    if "-opt" not in spine_path or "-fastbuild" in spine_path:
        raise CompilationModeError(
            "This example is meant to be called by ``bazel run -c opt`` "
            "so that the simulator performs well, but it seems the "
            'compilation mode is "fastbuild"? Go to the code and comment out '
            "this check if you know what you are doing :)"
        )

    # Gin configuration
    gin.parse_config_file(f"{agent_dir}/config/common.gin")
    gin.parse_config_file(f"{agent_dir}/config/bullet.gin")

    # Spine configuration
    with open(f"{agent_dir}/config/spine.yaml", "r") as fh:
        config = yaml.safe_load(fh)

    pid = os.fork()
    if pid == 0:  # child process: spine
        spine_argv = ["--spine-frequency", "1000.0", "--show"]
        os.execvp(spine_path, ["bullet"] + spine_argv)
    else:
        spine = None
        try:
            spine = SpineInterface(retries=10)
            run(spine, config)
        except KeyboardInterrupt:
            logging.info("Caught a keyboard interrupt")
        except Exception:
            logging.error("Controller raised an exception")
            print("")
            traceback.print_exc()
            print("")
        finally:
            if spine:
                spine.stop()
            os.kill(pid, signal.SIGINT)  # interrupt spine child process
            os.waitpid(pid, 0)  # wait for spine to terminate
