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
        remote = "https://github.com/ubgk/vulp.git",
        commit = "cc2f7c5a83c018f9982c615f25dbf41c5e87e058",
        shallow_since = "1714667492 +0200",
    )
