import cv2
import numpy as np
import torch
import math
from ultralytics import YOLO
from datetime import datetime

# RealSense SDK
import pyrealsense2 as rs

# ROS2
import rclpy
from rclpy.node import Node
from arm_interfaces.srv import MoveArm   # 너가 만든 서비스
from geometry_msgs.msg import Point

CONF_THRES = 0.35
IOU_THRES  = 0.7

class VisionArmClient(Node):
    def __init__(self):
        super().__init__('vision_arm_client')
        self.cli = self.create_client(MoveArm, 'move_arm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_arm 서비스 대기 중...')

        # YOLO 모델
        self.model = YOLO("best.pt").to('cuda:0' if torch.cuda.is_available() else 'cpu')

        # RealSense 파이프라인
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)

        # 카메라 intrinsics
        depth_sensor = self.profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_sensor.as_video_stream_profile().get_intrinsics()

        self.cx_img = self.intrinsics.width // 2
        self.cy_img = self.intrinsics.height // 2

    def pixel_to_3d(self, u, v, depth):
        """픽셀(u,v) + 깊이 → 3D 좌표 [m]"""
        X = (u - self.intrinsics.ppx) / self.intrinsics.fx * depth
        Y = (v - self.intrinsics.ppy) / self.intrinsics.fy * depth
        Z = depth
        return X, Y, Z

    def detect_and_act(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        img = np.asanyarray(color_frame.get_data())

        results = self.model.predict(img, conf=CONF_THRES, iou=IOU_THRES, verbose=False)
        if len(results) == 0:
            return

        r0 = results[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return

        # 최고 conf 물체 1개 선택
        xyxy = r0.boxes.xyxy.cpu().numpy()
        confs = r0.boxes.conf.cpu().numpy()
        top_idx = int(np.argmax(confs))
        x1, y1, x2, y2 = map(int, xyxy[top_idx])

        # 바운딩박스 중심 픽셀
        u = (x1 + x2) // 2
        v = (y1 + y2) // 2

        depth = depth_frame.get_distance(u, v)  # [m]
        if depth <= 0:
            self.get_logger().warn("깊이값 없음")
            return

        X, Y, Z = self.pixel_to_3d(u, v, depth)
        self.get_logger().info(f"Target 3D: X={X:.3f} Y={Y:.3f} Z={Z:.3f}")

        # 서비스 호출
        req = MoveArm.Request()
        req.x = float(X)
        req.y = float(Y)
        req.z = float(Z)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"MoveArm 결과: {future.result().success}, {future.result().message}")
        else:
            self.get_logger().error("MoveArm 서비스 호출 실패")

def main():
    rclpy.init()
    node = VisionArmClient()
    try:
        while rclpy.ok():
            node.detect_and_act()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
