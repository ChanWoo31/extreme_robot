#!/usr/bin/env python3

import cv2
import time
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import subprocess
import sys

# ---------- 설정 ----------
class RealSense:
    def __init__(self):
        self.TIMEOUT_MS = 150000 # milliseconds
        self.WIDTH, self.HEIGHT, self.FPS = 640, 480, 30
        self.USE_FILTERS = True
        self.WINDOW_NAME = "RealSense Depth (press 'q' to quit)"

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.FPS)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)
        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.colorizer = rs.colorizer()

        self.filters = self.build_filters() if self.USE_FILTERS else None
        self.intrinsics, (self.xx_factor, self.yy_factor) = self.get_intrinsic()

        # YOLO 가중치 경로 수정
        self.yolo = YOLO('/home/han/Documents/best.pt')

    def stop(self):
        self.pipeline.stop()

    def build_filters(self):
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 2)
        temporal = rs.temporal_filter()
        hole_filling = rs.hole_filling_filter(1)
        return [spatial, temporal, hole_filling]

    def apply_filters(self, depth_frame):
        spatial, temporal, hole_filling = self.filters
        f = spatial.process(depth_frame)
        f = temporal.process(f)
        f = hole_filling.process(f)
        return f

    def get_frames(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=self.TIMEOUT_MS)
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        return color_frame, depth_frame

    def get_intrinsic(self):
        try:
            while True:
                color_frame, depth_frame = self.get_frames()
                if not depth_frame or not color_frame:
                    continue
                intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
                fx, fy = intrinsics.fx, intrinsics.fy
                cx, cy = intrinsics.ppx, intrinsics.ppy
                u = np.arange(self.WIDTH, dtype=np.float32)
                v = np.arange(self.HEIGHT, dtype=np.float32)
                uu, vv = np.meshgrid(u, v) # (H, W)
                xx_factor = (uu - cx) / fx # X = xx_factor * Z
                yy_factor = (vv - cy) / fy # Y = yy_factor * Z
                return intrinsics, (xx_factor, yy_factor)
        except Exception:
            self.pipeline.stop()
            raise

    def get_point_coord(self, depth_img_m, point_uv):
        u, v = int(point_uv[0]), int(point_uv[1])
        u0, v0 = max(0, u - 1), max(0, v - 1)
        u1, v1 = min(self.WIDTH, u + 2), min(self.HEIGHT, v + 2)
        crop = depth_img_m[v0:v1, u0:u1]
        valid = crop[np.isfinite(crop) & (crop > 0)]
        z = float(np.median(valid)) if valid.size > 0 else 0.0 # meters
        X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z)
        return np.array([X, Y, Z], dtype=np.float32), z

    @staticmethod
    def cam_to_common(XYZ_cam: np.ndarray) -> np.ndarray:
        Xc, Yc, Zc = float(XYZ_cam[0]), float(XYZ_cam[1]), float(XYZ_cam[2])
        return np.array([Zc, -Xc, -Yc], dtype=np.float32)


def stream_to_virtual_camera():
    """FFmpeg를 사용하여 가상 카메라로 스트리밍"""
    rsys = RealSense()
    proc = None

    # 간단한 테스트를 위해 GStreamer 사용
    gst_cmd = [
        'gst-launch-1.0',
        'fdsrc',
        '!', 'videoparse', 'width=1280', 'height=480', 'format=bgr',
        '!', 'videoconvert',
        '!', 'v4l2sink', 'device=/dev/video10'
    ]

    try:
        proc = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        print("GStreamer로 가상 카메라 스트리밍 시작...")

        frame_count = 0
        while True:
            color_frame, depth_frame = rsys.get_frames()
            if not depth_frame or not color_frame:
                continue

            if rsys.filters is not None:
                depth_frame = rsys.apply_filters(depth_frame)

            depth_raw = np.asanyarray(depth_frame.get_data())
            color_bgr = np.asanyarray(color_frame.get_data())

            depth_m = depth_raw.astype(np.float32) * rsys.depth_scale
            depth_m[depth_raw == 0] = np.nan

            result = rsys.yolo.predict(source=color_bgr, verbose=False, conf=0.7)[0]
            vis = color_bgr.copy()

            if result.boxes is not None and len(result.boxes) > 0:
                xyxy = result.boxes.xyxy.cpu().numpy()
                cls = result.boxes.cls.cpu().numpy()
                conf = result.boxes.conf.cpu().numpy()
                for box, cls_id, c in zip(xyxy, cls, conf):
                    x1, y1, x2, y2 = box.astype(int)
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2
                    (XYZ_cam, z) = rsys.get_point_coord(depth_m, (u, v))
                    XYZ_common = rsys.cam_to_common(XYZ_cam)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(vis, (u, v), 3, (0, 0, 255), -1)
                    name = rsys.yolo.names[int(cls_id)] if hasattr(rsys.yolo, "names") else str(int(cls_id))
                    x, y, z = XYZ_common
                    cv2.putText(vis, f"{name} {c:.2f} x={x:.3f} y={y:.3f} z={z:.3f}",
                                (x1, max(0, y1-7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            depth_vis = np.asanyarray(rsys.colorizer.colorize(depth_frame).get_data())
            stack = np.hstack([vis, depth_vis])   # 스트리밍할 프레임

            # GStreamer로 프레임 전송
            try:
                proc.stdin.write(stack.tobytes())
                proc.stdin.flush()
                frame_count += 1
                if frame_count % 30 == 0:
                    print(f"프레임 {frame_count} 전송됨")
            except BrokenPipeError:
                print("GStreamer 프로세스 종료됨")
                break

            time.sleep(1/30)  # 30 FPS 유지

    except Exception as e:
        print(f"오류 발생: {e}")
        # 다시 pyvirtualcam 시도
        try:
            print("pyvirtualcam으로 다시 시도...")
            import pyvirtualcam

            frame_count = 0
            with pyvirtualcam.Camera(width=1280, height=480, fps=30, device='/dev/video10') as cam:
                print("pyvirtualcam 가상 카메라 생성 성공!")
                while True:
                    color_frame, depth_frame = rsys.get_frames()
                    if not depth_frame or not color_frame:
                        continue

                    if rsys.filters is not None:
                        depth_frame = rsys.apply_filters(depth_frame)

                    depth_raw = np.asanyarray(depth_frame.get_data())
                    color_bgr = np.asanyarray(color_frame.get_data())

                    depth_m = depth_raw.astype(np.float32) * rsys.depth_scale
                    depth_m[depth_raw == 0] = np.nan

                    result = rsys.yolo.predict(source=color_bgr, verbose=False, conf=0.7)[0]
                    vis = color_bgr.copy()

                    if result.boxes is not None and len(result.boxes) > 0:
                        xyxy = result.boxes.xyxy.cpu().numpy()
                        cls = result.boxes.cls.cpu().numpy()
                        conf = result.boxes.conf.cpu().numpy()
                        for box, cls_id, c in zip(xyxy, cls, conf):
                            x1, y1, x2, y2 = box.astype(int)
                            u = (x1 + x2) // 2
                            v = (y1 + y2) // 2
                            (XYZ_cam, z) = rsys.get_point_coord(depth_m, (u, v))
                            XYZ_common = rsys.cam_to_common(XYZ_cam)
                            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(vis, (u, v), 3, (0, 0, 255), -1)
                            name = rsys.yolo.names[int(cls_id)] if hasattr(rsys.yolo, "names") else str(int(cls_id))
                            x, y, z = XYZ_common
                            cv2.putText(vis, f"{name} {c:.2f} x={x:.3f} y={y:.3f} z={z:.3f}",
                                        (x1, max(0, y1-7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    depth_vis = np.asanyarray(rsys.colorizer.colorize(depth_frame).get_data())
                    stack = np.hstack([vis, depth_vis])   # 스트리밍할 프레임

                    # pyvirtualcam으로 프레임 전송 - RGB 변환 필요
                    cam.send(cv2.cvtColor(stack, cv2.COLOR_BGR2RGB))
                    cam.sleep_until_next_frame()

                    frame_count += 1
                    if frame_count % 30 == 0:
                        print(f"프레임 {frame_count} 전송됨")

        except Exception as e2:
            print(f"pyvirtualcam도 실패: {e2}")

    except KeyboardInterrupt:
        print("스트리밍 중단...")
    finally:
        if proc:
            proc.stdin.close()
            proc.terminate()
            proc.wait()
        rsys.stop()
        print("스트리밍 종료")


if __name__ == "__main__":
    print("RealSense + YOLO 가상 카메라 스트리밍 시작")
    print("Ctrl+C로 종료")
    stream_to_virtual_camera()