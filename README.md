# Upkie wheeled biped robot

[![CI](https://github.com/upkie/upkie/actions/workflows/bazel.yml/badge.svg)](https://github.com/upkie/upkie/actions/workflows/bazel.yml)
[![Build instructions](https://img.shields.io/badge/build-read-brightgreen?logo=read-the-docs&style=flat)](https://github.com/upkie/upkie/wiki)
[![Documentation](https://img.shields.io/badge/docs-read-brightgreen?logo=read-the-docs&style=flat)](https://upkie.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/upkie/upkie/badge.svg?branch=main)](https://coveralls.io/github/upkie/upkie?branch=main)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)
[![Chat](https://img.shields.io/badge/matrix-chat-%234eb899)](https://app.element.io/#/room/#tasts-robots:matrix.org)

**Upkie** is a fully open source self-balancing wheeled biped robot. It has wheels for balancing, and legs to go off-road or negotiate uneven terrains. Upkies are designed to be buildable at home with tools and components ordered online, like mjbots actuators. Motion control runs onboard the robot's Raspberry Pi.

This repository contains all the software and instructions required to build and operate an Upkie. Development can be done in Python or C++, on Linux or macOS. Questions are welcome in the [Chat](https://app.element.io/#/room/#tasts-robots:matrix.org) and [Discussions forum](https://github.com/upkie/upkie/discussions).

## Installation

To code for Upkie in Python, just install:

```console
pip install upkie
```

This Python interface is already [fast enough](https://github.com/upkie/vulp#performance) for real-time control. If later on you want to optimize parts of your code, you can move them to C++ [spines](https://upkie.github.io/upkie/spines.html).

## Simulation

Assuming you have a C++ compiler (setup one-liners: [Fedora](https://github.com/upkie/upkie/discussions/100), [Ubuntu](https://github.com/upkie/upkie/discussions/101)), you can run an Upkie simulation right from the command line. It won't install anything on your machine, everything will run locally from the repository:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/upkie/upkie.git
cd upkie
./start_pid_balancer.sh
```

Click on the robot in the simulator window to apply external forces.

## Example

You can develop your own agent using the Gymnasium environments distributed in ``upkie.envs``. For example, here is a simple proportional-feedback balancer:

```python
import gymnasium as gym
import upkie.envs

upkie.envs.register()

with gym.make("UpkieGroundVelocity-v3", frequency=200.0) as env:
    observation = env.reset()
    action = 0.0 * env.action_space.sample()
    for step in range(1_000_000):
        observation, reward, done, _ = env.step(action)
        if done:
            observation = env.reset()
        pitch = observation[0]
        action[0] = 10.0 * pitch
```

To test this agent on your computer, run the agent and simulation spine in two separate processes: `python this_agent.py` in one shell, and `./start_simulation.sh` in the other.

To run this agent on the robot, `scp` the script to the Raspberry Pi, start a [pi3hat spine](https://upkie.github.io/upkie/spines.html#pi3hat-spine) and execute the script on the Pi itself.

## To go further

- [Build your own Upkie](https://github.com/upkie/upkie/wiki) 🧰
- [Documentation](https://upkie.github.io/upkie/)
- [More examples](https://github.com/upkie/upkie/tree/main/examples)
- [Project log on Hackaday.io](https://hackaday.io/project/185729-upkie-wheeled-biped-robots)
