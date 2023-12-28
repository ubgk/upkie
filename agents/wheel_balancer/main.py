#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

import os
# Check if MSGPACK_PUREPYTHON environment variable is set
if os.environ.get("MSGPACK_PUREPYTHON") is None:
    print("MSGPACK_PUREPYTHON environment variable is not set")
else:
    print("MSGPACK_PUREPYTHON environment variable is already set to 1")

import msgpack
print("msgpack imported successfully", msgpack, msgpack.Unpacker)
    

import argparse
import socket
import traceback
from os import path
from typing import Any, Dict

import gin
import yaml
from loop_rate_limiters import RateLimiter
from servo_controller import ServoController
from vulp.spine import SpineInterface

from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging

import time

import cProfile

def parse_command_line_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns:
        Command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        metavar="config",
        help="Agent configuration to apply",
        type=str,
        required=True,
    )
    return parser.parse_args()

import time
def run(
    spine: SpineInterface,
    spine_config: Dict[str, Any],
    frequency: float = 200.0,
) -> None:
    """
    Read observations and send actions to the spine.

    Args:
        spine: Interface to the spine.
        spine_config: Spine configuration dictionary.
        frequency: Control frequency in Hz.
    """
    controller = ServoController()
    debug: Dict[str, Any] = {}
    dt = 1.0 / frequency
    rate = RateLimiter(frequency, "controller")

    wheel_radius = controller.wheel_balancer.wheel_radius
    spine_config["wheel_odometry"] = {
        "signed_radius": {
            "left_wheel": +wheel_radius,
            "right_wheel": -wheel_radius,
        }
    }

    spine.start(spine_config)
    observation = spine.get_observation()  # pre-reset observation
    while True:
        start = time.time()
        observation = spine.get_observation()
        end = time.time()
        delta = end - start
        delta_ms = delta * 1000
        
        obs_str = f"[Observation] time: {delta_ms:.4f} ms"
        action = controller.cycle(observation, dt)

        spine.set_action(action)
        debug["rate"] = {"slack": rate.slack}
        
        end = time.time()
        delta = end - start
        delta_ms = delta * 1000
        print(obs_str)
        print(f"[Cycle] time: {delta_ms:.4f} ms")
        
        rate.sleep()


def load_gin_configuration(base_dir: str, path: str) -> None:
    logging.info(f"Loading configuration '{path}'")
    try:
        gin.parse_config_file(f"{base_dir}/{path}")
    except OSError as e:
        raise FileNotFoundError(f"Configuration '{path}' not found") from e


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()

    args = parse_command_line_arguments()
    agent_dir = path.dirname(__file__)

    # Load gin configuration files
    load_gin_configuration(agent_dir, "config/common.gin")
    if args.config == "hostname":
        hostname = socket.gethostname().lower()
        logging.info(f"Loading configuration from hostname '{hostname}'")
        load_gin_configuration(agent_dir, f"config/{hostname}.gin")
    elif args.config is not None:
        load_gin_configuration(agent_dir, f"config/{args.config}.gin")

    # Spine configuration
    with open(f"{agent_dir}/config/spine.yaml", "r") as fh:
        spine_config = yaml.safe_load(fh)

    spine = SpineInterface()
    try:
        cProfile.run("run(spine, spine_config)")
        # run(spine, spine_config)
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")
    except Exception:
        logging.error("Controller raised an exception")
        print("")
        traceback.print_exc()
        print("")

    logging.info("Stopping the spine...")
    try:
        spine.stop()
    except Exception:
        logging.error("Error while stopping the spine!")
        print("")
        traceback.print_exc()
        print("")
