package(default_visibility = ["//visibility:public"])

licenses(["notice"])

load("@rules_proto//proto:defs.bzl", "cc_proto_library")

cc_proto_library(
    name = "edge_follow_path_cc_proto",
    deps = [":edge_follow_path_proto"],
)

proto_library(
    name = "edge_follow_path_proto",
    srcs = ["edge_follow_path.proto"],
    deps = ["//modules/planning/planning_base/proto:piecewise_jerk_path_config_proto"],
)
