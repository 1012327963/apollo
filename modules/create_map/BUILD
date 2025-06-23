load("//tools/proto:proto.bzl", "apollo_py_binary")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "collect_pcd",
    srcs = ["collect_pcd.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/transform_msgs:transform_py_pb2",
        "//modules/common_msgs/sensor_msgs:imu_py_pb2",
    ],
)

apollo_package()
