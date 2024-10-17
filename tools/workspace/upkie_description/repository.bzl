# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def upkie_description_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "upkie_description",
        remote = "https://github.com/ubgk/upkie_description",
        commit = "2e1d5fbfebe7481c3dc14d6a88f79309136d7cfd"
    )
