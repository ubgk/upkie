#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0


from typing import Callable


def affine_schedule(y_0: float, y_1: float) -> Callable[[float], float]:
    """!
    Affine schedule as a function over the [0, 1] interval.

    @param y_0 Function value at zero.
    @param y_1 Function value at one.
    @return Corresponding affine function.
    """
    diff = y_1 - y_0

    def schedule(x: float) -> float:
        """!
        Compute the current learning rate from remaining progress.

        @param x Progress decreasing from 1 (beginning) to 0.
        @return Corresponding learning rate>
        """
        return y_0 + x * diff

    return schedule


def exponential_decay_schedule(
    initial_value: float,
    nb_phases: int,
    factor: float = 0.1,
) -> Callable[[float], float]:
    """!
    Step-by-step exponential-decay learning rate schedule.

    @param initial_value Learning rate at the beginning of training.
    @param nb_phases Number of different phases.
    @param factor Initial value is multiplied by this factor at each successive
        phase.
    @return Function computing the current learning rate from remaining
        progress.
    """

    def schedule(progress_remaining: float) -> float:
        """!
        Compute the current learning rate from remaining progress.

        @param progress_remaining Progress decreasing from 1 (beginning) to 0.
        @return Corresponding learning rate>
        """
        progress: float = 1.0 - progress_remaining
        phase = int(nb_phases * progress)
        return initial_value * factor**phase

    return schedule
