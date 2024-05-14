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
        commit = "5ef2524e280aaa9d0153b8ecdf7b9dcab57a878b",
        shallow_since = "1714667492 +0200",
    )
