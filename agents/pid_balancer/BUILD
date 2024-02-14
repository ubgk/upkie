# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "controllers",
    srcs = [
        "servo_controller.py",
        "wheel_controller.py",
    ],
    deps = [
        "//upkie/observers/base_pitch",
        "//upkie/utils:clamp",
        "//upkie/utils:filters",
        "//upkie/utils:contact_models",
        requirement("gin-config"),
    ],
)

py_binary(
    name = "bullet",
    srcs = ["bullet.py"],
    main = "bullet.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        ":config",
        "//spines:bullet_spine",
    ],
    deps = [
        "//upkie/utils:exceptions",
        ":controllers",
        "@rules_python//python/runfiles",
        "@vulp//:python",
    ],
)

py_binary(
    name = "pid_balancer",
    srcs = ["main.py"],
    main = "main.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        "//agents/pid_balancer/config",
    ] + select({
        "//:pi64_config": ["//spines:pi3hat_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "//upkie/utils:exceptions",
        "//upkie/utils:raspi",
        "//upkie/utils:spdlog",
        ":controllers",
        "@vulp//:python",
        requirement("loop_rate_limiters"),
        # requirement("mpacklog"),
        requirement("pyyaml"),
    ],
)

add_lint_tests()
