#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""Collects point cloud and localization data to generate a PCD map."""

import numpy as np
import open3d as o3d
import cv2
from cyber.python.cyber_py3 import cyber
from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointCloud
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStampeds
from modules.common_msgs.sensor_msgs.imu_pb2 import Imu
from scipy.spatial.transform import Rotation
from collections import defaultdict
from bisect import bisect_left

accumulated_pcd = None
first_xyz = None
got_pose = False
cur_pose = None
last_pose = None
min_add_scan_shift = 10
voxel_size = 0.001

# TF history: {child_frame_id: [(timestamp, (t, q)), ...]}
tf_history = defaultdict(list)
MAX_TF_HISTORY = 2000
# IMU buffer
imu_buffer = []  # [(timestamp, linear_acc, angular_vel), ...]

# Lever-arm offset of lidar relative to IMU
lever_arm = np.array([0.0, 0.5, -0.5])

def interpolate_tf(t1, t2, q1, q2, target_time):
    """Linear interpolation between two transforms."""
    t = t1[1] + (target_time - t1[0]) / (t2[0] - t1[0]) * (t2[1] - t1[1])
    q = q1[1] + (target_time - t1[0]) / (t2[0] - t1[0]) * (q2[1] - q1[1])
    q /= np.linalg.norm(q)
    return t, q

def tf_callback(msg: TransformStampeds):
    for tf in msg.transforms:
        timestamp = tf.header.timestamp_sec
        t = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=np.float64)
        q = np.array([
            tf.transform.rotation.qw,
            tf.transform.rotation.qx,
            tf.transform.rotation.qy,
            tf.transform.rotation.qz,
        ], dtype=np.float64)
        q /= np.linalg.norm(q)
        child_frame = tf.child_frame_id
        tf_history[child_frame].append((timestamp, (t, q)))
        if len(tf_history[child_frame]) > MAX_TF_HISTORY:
            tf_history[child_frame].pop(0)
        tf_history[child_frame].sort(key=lambda x: x[0])
        print(
            f"Received TF: parent_frame={tf.header.frame_id}, child_frame={child_frame}, "
            f"timestamp={timestamp:.3f}, t={t.tolist()}, q={q.tolist()}"
        )

def imu_callback(msg: Imu):
    timestamp = msg.measurement_time
    linear_acc = np.array([
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
    ], dtype=np.float64)
    angular_vel = np.array([
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
    ], dtype=np.float64)
    imu_buffer.append((timestamp, linear_acc, angular_vel))
    if len(imu_buffer) > 1000:
        imu_buffer.pop(0)
    imu_buffer.sort(key=lambda x: x[0])

def find_closest_tf(child_frame_id, target_time):
    if child_frame_id not in tf_history or not tf_history[child_frame_id]:
        return None
    timestamps = [x[0] for x in tf_history[child_frame_id]]
    idx = bisect_left(timestamps, target_time)
    if idx == 0:
        return tf_history[child_frame_id][0][1]
    if idx == len(timestamps):
        return tf_history[child_frame_id][-1][1]
    prev_time, prev_tf = tf_history[child_frame_id][idx - 1]
    next_time, next_tf = tf_history[child_frame_id][idx]
    if abs(target_time - prev_time) < abs(target_time - next_time):
        return prev_tf
    return prev_tf

def find_closest_imu(target_time):
    if not imu_buffer:
        return None
    timestamps = [x[0] for x in imu_buffer]
    idx = bisect_left(timestamps, target_time)
    if idx == 0:
        return imu_buffer[0]
    if idx == len(timestamps):
        return imu_buffer[-1]
    prev_time, _, _ = imu_buffer[idx - 1]
    next_time, _, _ = imu_buffer[idx]
    if abs(target_time - prev_time) < abs(target_time - next_time):
        return imu_buffer[idx - 1]
    return imu_buffer[idx]

def localization_callback(msg):
    global cur_pose, first_xyz, got_pose
    t = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ], dtype=np.float64)
    q = np.array([
        msg.pose.orientation.qw,
        msg.pose.orientation.qx,
        msg.pose.orientation.qy,
        msg.pose.orientation.qz,
    ], dtype=np.float64)
    q /= np.linalg.norm(q)
    cur_pose = (t, q)
    if not got_pose:
        first_xyz = (t[0], t[1], t[2])
        got_pose = True
        print(
            f"[First Localization] GNSS origin: x={t[0]:.3f}, y={t[1]:.3f}, z={t[2]:.3f}"
        )

def pointcloud_callback(msg):
    global accumulated_pcd, cur_pose, last_pose
    timestamp = msg.header.timestamp_sec
    print(f"Received point cloud, ts: {timestamp:.3f}, frame_id: {msg.header.frame_id}")
    if cur_pose is None:
        print("Localization not yet received, skip frame")
        return
    t_tf, q_tf = cur_pose
    rot = Rotation.from_quat([q_tf[1], q_tf[2], q_tf[3], q_tf[0]])
    lever_offset = rot.apply(lever_arm)
    t_corrected = t_tf + lever_offset
    print(
        f"Using pose, t={t_tf.tolist()}, q={q_tf.tolist()}, corrected t={t_corrected.tolist()}"
    )
    xyz = np.array([[p.x, p.y, p.z] for p in msg.point], dtype=np.float32)
    xyz = xyz[~np.isnan(xyz).any(axis=1)]
    if xyz.shape[0] == 0:
        print("Empty point cloud, skip")
        return
    xyz_rot = rot.apply(xyz)
    xyz_world = xyz_rot + t_corrected
    print(
        f"Point cloud range: x({np.min(xyz_world[:,0]):.2f},{np.max(xyz_world[:,0]):.2f}), "
        f"y({np.min(xyz_world[:,1]):.2f},{np.max(xyz_world[:,1]):.2f})"
    )
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(xyz_world)
    if last_pose is not None:
        displacement = np.linalg.norm(t_corrected - last_pose)
        print(f"Displacement: {displacement:.3f}, threshold: {min_add_scan_shift}")
        if displacement < min_add_scan_shift:
            print("Movement too small, skip frame")
            return
    last_pose = t_corrected.copy()
    if accumulated_pcd is None:
        accumulated_pcd = new_pcd
    else:
        accumulated_pcd += new_pcd
    print(f"Accumulated points: {len(accumulated_pcd.points)}")
    if len(accumulated_pcd.points) % 100 == 0:
        o3d.io.write_point_cloud("temp_pcd.pcd", accumulated_pcd)
        print("Temporary point cloud saved to temp_pcd.pcd")

def save_results(
    output_pcd="/apollo_workspace/modules/tools/ndt_mapping/output_map.pcd",
    output_img="/apollo_workspace/modules/tools/ndt_mapping/output_map_cv.jpg",
    gnss_offset_file="/apollo_workspace/modules/tools/ndt_mapping/gnss-map-offset.txt",
    transdata_file="/apollo_workspace/modules/tools/ndt_mapping/transData.txt",
):
    global accumulated_pcd, first_xyz
    if accumulated_pcd is None or len(accumulated_pcd.points) == 0:
        print("No point cloud data to save.")
        return
    pcd_down = accumulated_pcd.voxel_down_sample(voxel_size=0.1)
    pc_xyz_down = np.asarray(pcd_down.points)
    print(f"Accumulated points after downsample: {pc_xyz_down.shape[0]}")
    o3d.io.write_point_cloud(output_pcd, pcd_down)
    print(f"PCD saved to: {output_pcd}")
    min_x, min_y = pcd_to_image(pc_xyz_down, output_img)
    with open(transdata_file, "w") as f:
        f.write(f"{min_x}  {min_y}  10\n")
    print(f"Bounds saved to: {transdata_file}")
    if first_xyz is not None:
        with open(gnss_offset_file, "w") as f:
            f.write(f"{first_xyz[0]} {first_xyz[1]} {first_xyz[2]}\n")
        print(f"GNSS origin saved to: {gnss_offset_file}")
    else:
        print("No localization received, gnss offset not saved.")

def pcd_to_image(pc_xyz, output_img="output_map_cv.jpg"):
    min_x, max_x = np.min(pc_xyz[:, 0]), np.max(pc_xyz[:, 0])
    min_y, max_y = np.min(pc_xyz[:, 1]), np.max(pc_xyz[:, 1])
    print(f"Bounds: x({min_x:.2f},{max_x:.2f}), y({min_y:.2f},{max_y:.2f})")
    scale = 10
    width = int(np.ceil(max_x - min_x)) * scale + 1
    height = int(np.ceil(max_y - min_y)) * scale + 1
    image = np.ones((height, width, 3), dtype=np.uint8) * 255
    plotted_pixels = set()
    for pt in pc_xyz:
        x, y = pt[0], pt[1]
        px = int((x - min_x) * scale)
        py = int((max_y - y) * scale)
        key = (px, py)
        if key in plotted_pixels:
            continue
        plotted_pixels.add(key)
        color = (160, 160, 160)
        cv2.rectangle(image, (px - 1, py - 1), (px + 1, py + 1), color, thickness=-1)
    cv2.imwrite(output_img, image)
    print(f"Top view image saved to: {output_img}")
    return min_x, min_y

def main():
    cyber.init()
    node = cyber.Node("ndt_mapping_subscriber")
    node.create_reader("/tf", TransformStampeds, tf_callback)
    node.create_reader(
        "/apollo/localization/pose", LocalizationEstimate, localization_callback
    )
    node.create_reader(
        "/apollo/sensor/lidar16/back/PointCloud2", PointCloud, pointcloud_callback
    )
    node.create_reader("/apollo/sensor/gnss/imu", Imu, imu_callback)
    print("Collecting point cloud and GNSS origin, press Ctrl+C to stop...")
    try:
        node.spin()
    except KeyboardInterrupt:
        print("\nCollection stopped, saving files...")
    save_results()
    cyber.shutdown()

if __name__ == "__main__":
    main()
