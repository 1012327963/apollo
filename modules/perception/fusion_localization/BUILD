load("//tools:apollo_package.bzl", "apollo_package", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libfusion_localization_component.so",
    srcs = ["fusion_localization_component.cc"],
    hdrs = ["fusion_localization_component.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/perception/fusion_localization/proto:fusion_localization_cc_proto",
    ],
)

apollo_package()
cpplint()
