# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def vulp_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "vulp",
        remote = "https://github.com/upkie/vulp.git",
        commit = "b6bab0033f8ae12510a7ee381dd5b00825bfd949",
        shallow_since = "1716899811 +0200",
    )
