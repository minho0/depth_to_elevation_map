#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiImageViewer(Node):
    def __init__(self):
        super().__init__('multi_image_viewer')
        self.bridge = CvBridge()

        self.image_topics = {
            'frontleft': '/spot1/base/spot/camera/frontleft/image',
            'frontright': '/spot1/base/spot/camera/frontright/image',
            'depthleft': '/spot1/base/spot/depth/frontleft/image',
            'depthright': '/spot1/base/spot/depth/frontright/image',
        }

        self.images = {name: None for name in self.image_topics}

        for name, topic in self.image_topics.items():
            self.create_subscription(Image, topic, lambda msg, n=name: self.callback(msg, n), 10)

    def callback(self, msg, name):
        try:
            is_depth = 'depth' in name

            if is_depth:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  # 16UC1 그대로
                # 그대로 사용 (시각화용 변환 없음)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.images[name] = cv_image

            if all(img is not None for img in self.images.values()):
                self.display_images()
        except Exception as e:
            self.get_logger().error(f"[{name}] Error: {e}")

    def display_images(self):
        # ── 1) 시각화용 BGR-uint8 형태로 정규화 및 회전 ────────────────────
        vis = {}
        for name, img in self.images.items():
            if 'depth' in name:
                img8 = (img / 256).astype(np.uint8)
                img_bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
            else:
                img_bgr = img.copy()
            
            # 시계방향으로 90도 회전
            vis[name] = cv2.rotate(img_bgr, cv2.ROTATE_90_CLOCKWISE)

        # ── 2) 해상도 맞추기 ─────────────────────────────────────────────
        h_min = min(v.shape[0] for v in vis.values())
        w_min = min(v.shape[1] for v in vis.values())
        for k in vis:
            vis[k] = cv2.resize(vis[k], (w_min, h_min), interpolation=cv2.INTER_AREA)

        # ── 3) 2×2 타일로 합치기 ─────────────────────────────────────────
        row1 = cv2.hconcat([vis['frontright'],  vis['frontleft']])
        row2 = cv2.hconcat([vis['depthright'],  vis['depthleft']])
        grid = cv2.vconcat([row1, row2])

        # ── 4) 출력 ─────────────────────────────────────────────────────
        cv2.imshow("image_view", grid)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = MultiImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()