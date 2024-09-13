# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def mpacklog_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    http_archive(
        name = "mpacklog",
        sha256 = "94b613c484079dbb239f4dde6ec041e8696399b4a6fc68f22b4633d1fa561f2f",
        strip_prefix = "mpacklog.cpp-3.1.0",
        url = "https://github.com/ubgk/mpacklog.cpp/archive/refs/tags/v3.1.0.tar.gz",
    )
