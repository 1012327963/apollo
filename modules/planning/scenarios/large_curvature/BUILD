load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "liblarge_curvature_scenario.so",
    srcs = [
        "large_curvature_scenario.cc",
        "large_curvature_stage.cc",
    ],
    hdrs = [
        "large_curvature_scenario.h",
        "large_curvature_stage.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/large_curvature/proto:large_curvature_cc_proto",
    ],
)

apollo_package()

cpplint()
