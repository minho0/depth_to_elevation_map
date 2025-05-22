#!/usr/bin/env python3
"""ROS 2 node: depth image → PointCloud2 (body frame)
   * Depth image topics (frontleft, frontright)
   * Depth encoding: 16UC1 (mm) → meters
   * Mask any value > 10 m
   * TF lookup first → fallback to YAML extrinsic
"""
import os
import struct
import yaml
from typing import Dict, Optional

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from transforms3d.quaternions import quat2mat
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException, ExtrapolationException  # TF 관련 모듈
from geometry_msgs.msg import TransformStamped

###############################################################################
# Utility
###############################################################################

def load_extrinsic_matrix(yaml_filename: str, key: str) -> np.ndarray:
    """Load 4×4 extrinsic matrix from YAML in package *depth_to_pointcloud_pub*"""
    package_share = get_package_share_directory("depth_to_pointcloud_pub")
    path = os.path.join(package_share, "config", yaml_filename)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return np.array(data[key]["Matrix"]).reshape(4, 4)

###############################################################################
# Main Node
###############################################################################

class DepthToPointCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_to_pointcloud_node")

        # ── Configuration ────────────────────────────────────────────────────
        self.camera_prefixes = ["frontleft", "frontright"]
        self.depth_base = "/spot1/base/spot/depth"
        self.output_ns = "/pointcloud"   # published topic: /pointcloud/<prefix>
        self.body_frame = "spot1/base/spot/body" 
        self.synced_frame = f"{self.body_frame}/synced"


        # ── State ────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.K_mats: Dict[str, Optional[np.ndarray]] = {p: None for p in self.camera_prefixes}
        self.extrinsics = {
            # "frontleft": np.linalg.inv(load_extrinsic_matrix("frontleft_info.yaml", "body_to_frontleft")),
            # "frontright": np.linalg.inv(load_extrinsic_matrix("frontright_info.yaml", "body_to_frontright")),
            "frontleft": load_extrinsic_matrix("frontleft_info.yaml", "body_to_frontleft"),
            "frontright": load_extrinsic_matrix("frontright_info.yaml", "body_to_frontright"),
        }

        # TF objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # merged cloud buffer (cleared every publication cycle)
        self._merge_buffer: Dict[str, np.ndarray] = {}

        # Subscribers
        for prefix in self.camera_prefixes:
            self.create_subscription(
                CameraInfo,
                f"{self.depth_base}/{prefix}/camera_info",
                lambda msg, p=prefix: self._camera_info_cb(msg, p),
                10,
            )
            self.create_subscription(
                Image,
                f"{self.depth_base}/{prefix}/image",
                lambda msg, p=prefix: self._depth_cb(msg, p),
                10,
            )

    # ───────────────────────────── Callbacks ────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo, prefix: str) -> None:
        self.K_mats[prefix] = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f"[{prefix}] CameraInfo received (fx={msg.k[0]:.1f}).", once=True)

    def _depth_cb(self, msg: Image, prefix: str) -> None:
        K = self.K_mats[prefix]
        if K is None:
            self.get_logger().warning(f"[{prefix}] CameraInfo not yet received.")
            return

        # 1️⃣ ROS Image (16UC1, mm) → numpy.uint16
        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # 2️⃣ mm → meters (float32) & 10m 초과는 0으로 마스킹
        depth_m = depth_raw.astype(np.float32) / 1000.0
        # depth_m[depth_m > 10.0] = 0.0  # mask
        depth_m[depth_m > 5.0] = 0.0  # mask

        # 3️⃣ Depth → 3‑D (camera frame)
        pts4 = self._depth_to_3d(depth_m, K)
        pts4 = pts4[:, pts4[2, :] > 0.1]   # keep z > 0.1 m

        # 4️⃣ Transform to body frame via TF or YAML
        T_cam_to_body = self.extrinsics[prefix]
        # pts_body = (T_cam_to_body @ pts4)[:3].T  # shape (N,3)

        pts_cam_body = T_cam_to_body @ pts4        # (4, N)
        pts_cam_body /= pts_cam_body[3, :]         # 마지막 행으로 나누기 (w로 나눔)
        pts_body = pts_cam_body[:3, :].T           # (N, 3)

        # 5️⃣ Build & publish PointCloud2
        pc_msg = self._create_pointcloud2(msg.header.stamp, self.body_frame, pts_body)
        self._publish_cloud(pc_msg, prefix)

        # 6️⃣ Accumulate for merged cloud
        self._merge_buffer[prefix] = pts_body  # overwrite latest pts for this camera
        if len(self._merge_buffer) == len(self.camera_prefixes):
            pts_merged = np.vstack(list(self._merge_buffer.values()))
            stamp = msg.header.stamp

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.body_frame  # parent = body
            tf_msg.child_frame_id = self.synced_frame  # child = body/synced
            tf_msg.transform.translation.x = 0.0  # identity 변환
            tf_msg.transform.translation.y = 0.0
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf_msg)

            pc_merged = self._create_pointcloud2(stamp, self.synced_frame, pts_merged)
            self._publish_cloud(pc_merged, "merged")
            self._merge_buffer.clear()



    # ───────────────────────────── Helpers ──────────────────────────────────

    @staticmethod
    def _depth_to_3d(depth: np.ndarray, K: np.ndarray) -> np.ndarray:
        h, w = depth.shape
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return np.vstack((x.ravel(), y.ravel(), z.ravel(), np.ones(z.size)))

    @staticmethod
    def _create_pointcloud2(stamp, frame: str, points: np.ndarray) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = frame  
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * points.shape[0]
        cloud.is_dense = True
        cloud.data = b"".join(struct.pack("fff", *pt) for pt in points)
        return cloud

    # publisher cache
    def _publish_cloud(self, msg: PointCloud2, prefix: str) -> None:
        if not hasattr(self, "_pub_cache"):
            self._pub_cache = {}
        topic = f"{self.output_ns}/{prefix}"
        if topic not in self._pub_cache:
            self._pub_cache[topic] = self.create_publisher(PointCloud2, topic, 10)
        self._pub_cache[topic].publish(msg)

###############################################################################
# Main entry
###############################################################################

def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
