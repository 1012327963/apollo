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
    name = "libedge_follow_path.so",
    srcs = ["edge_follow_path.cc"],
    hdrs = ["edge_follow_path.h"],
    copts = ["-DMODULE_NAME=\"planning\""] ,
    description = ":plugins.xml",
    deps = [
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/edge_follow_path/proto:edge_follow_path_cc_proto",
        "//modules/planning/tasks/lane_follow_path:liblane_follow_path.so",
    ],
)

apollo_package()

cpplint()
