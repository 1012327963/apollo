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
    name = "libedge_follow_scenario.so",
    srcs = [
        "edge_follow_scenario.cc",
        "edge_follow_stage.cc",
    ],
    hdrs = [
        "edge_follow_scenario.h",
        "edge_follow_stage.h",
    ],
    copts = ["-DMODULE_NAME=\"planning\""] ,
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_package()

cpplint()
