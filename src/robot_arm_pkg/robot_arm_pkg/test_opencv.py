import cv2
import numpy as np
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import pyrealsense2 as rs
from ultralytics import YOLO

# ---- ROS2 (rclpy) ----
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from arm_interfaces.msg import DetectedObject

# ---- mjpg-streamer ----
import subprocess
import shutil

# ---------- MJPEG 서버 (RealSense+YOLO+ColorTracking) ----------

LATEST_JPEG = None
JPEG_LOCK = threading.Lock()

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/stream.mjpg"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                with JPEG_LOCK:
                    frame = LATEST_JPEG
                if frame is None:
                    time.sleep(0.01)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(1/30)
        except (BrokenPipeError, ConnectionResetError):
            pass

def start_mjpeg_server(host="0.0.0.0", port=8080):
    httpd = ThreadingHTTPServer((host, port), MJPEGHandler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return httpd

class RealSenseColorTracker(Node):
    def __init__(self):
        super().__init__('realsense_color_tracker')

        # ROS2 퍼블리셔/서브스크라이버
        self.color_subscriber = self.create_subscription(String, '/target_color', self.color_callback, 10)
        self.direction_publisher = self.create_publisher(String, '/direction', 10)
        self.names_publisher = self.create_publisher(String, '/detected_object_names', 10)
        self.objects_publisher = self.create_publisher(DetectedObject, '/detected_objects', 10)

        # HSV 색상 범위 정의
        self.color_ranges = {
            'blue': [(100, 50, 50), (130, 255, 255)],      # 파랑
            'green': [(40, 50, 50), (80, 255, 255)],       # 초록
            'orange': [(10, 50, 50), (25, 255, 255)]       # 주황
        }

        self.target_color = None

        # RealSense 초기화
        self.init_realsense()

        # YOLO 초기화
        self.yolo = YOLO('/home/han/Documents/best.pt')

        self.timer = self.create_timer(0.03, self.process_frame)  # 30 FPS

    def color_callback(self, msg):
        color = msg.data.lower()
        if color in self.color_ranges:
            self.target_color = color
            self.get_logger().info(f'Target color set to: {color}')
        else:
            self.get_logger().warn(f'Unknown color: {color}. Available colors: blue, green, orange')

    def init_realsense(self):
        self.TIMEOUT_MS = 15000
        self.WIDTH, self.HEIGHT, self.FPS = 640, 480, 30
        self.USE_FILTERS = True

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.FPS)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)
        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        device = profile.get_device()
        color_sensor = device.first_color_sensor()
        if color_sensor.supports(rs.option.enable_auto_exposure):
            color_sensor.set_option(rs.option.enable_auto_exposure, True)
        if color_sensor.supports(rs.option.enable_auto_white_balance):
            color_sensor.set_option(rs.option.enable_auto_white_balance, True)

        self.align = rs.align(rs.stream.color)
        self.colorizer = rs.colorizer()

        self.filters = self.build_filters() if self.USE_FILTERS else None
        self.intrinsics, (self.xx_factor, self.yy_factor) = self.get_intrinsic()

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
        while True:
            color_frame, depth_frame = self.get_frames()
            if not depth_frame or not color_frame:
                continue
            intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
            fx, fy = intrinsics.fx, intrinsics.fy
            cx, cy = intrinsics.ppx, intrinsics.ppy
            u = np.arange(self.WIDTH, dtype=np.float32)
            v = np.arange(self.HEIGHT, dtype=np.float32)
            uu, vv = np.meshgrid(u, v)
            xx_factor = (uu - cx) / fx
            yy_factor = (vv - cy) / fy
            return intrinsics, (xx_factor, yy_factor)

    def get_point_coord(self, depth_img_m, point_uv):
        u, v = int(point_uv[0]), int(point_uv[1])
        u0, v0 = max(0, u - 1), max(0, v - 1)
        u1, v1 = min(self.WIDTH, u + 2), min(self.HEIGHT, v + 2)
        crop = depth_img_m[v0:v1, u0:u1]
        valid = crop[np.isfinite(crop) & (crop > 0)]
        z = float(np.median(valid)) if valid.size > 0 else 0.0
        X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z)
        return np.array([X, Y, Z], dtype=np.float32), z

    @staticmethod
    def cam_to_common(XYZ_cam: np.ndarray) -> np.ndarray:
        Xc, Yc, Zc = float(XYZ_cam[0]), float(XYZ_cam[1]), float(XYZ_cam[2])
        return np.array([Zc, -Xc, -Yc], dtype=np.float32)

    def find_centroid(self, bgr, low_hsv, high_hsv):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(low_hsv), np.array(high_hsv))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not cnts:
            return None

        # 가장 큰 컨투어 찾기
        c = max(cnts, key=cv2.contourArea)

        # 최소 면적 필터링
        if cv2.contourArea(c) < 500:
            return None

        M = cv2.moments(c)
        if M["m00"] == 0:
            return None

        return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

    def process_frame(self):
        try:
            color_frame, depth_frame = self.get_frames()
            if not depth_frame or not color_frame:
                return
        except Exception as e:
            self.get_logger().warn(f"RealSense frame error: {e}")
            return

        if self.filters is not None:
            depth_frame = self.apply_filters(depth_frame)

        depth_raw = np.asanyarray(depth_frame.get_data())
        color_bgr = np.asanyarray(color_frame.get_data())

        depth_m = depth_raw.astype(np.float32) * self.depth_scale
        depth_m[depth_raw == 0] = np.nan

        h, w = color_bgr.shape[:2]
        goal_x = w // 2  # 화면 중심 x좌표

        # YOLO 객체 감지
        result = self.yolo.predict(source=color_bgr, verbose=False, conf=0.7)[0]
        vis = color_bgr.copy()

        detected_names = []
        detected_objs = []

        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            cls = result.boxes.cls.cpu().numpy()
            conf = result.boxes.conf.cpu().numpy()
            for box, cls_id, c in zip(xyxy, cls, conf):
                x1, y1, x2, y2 = box.astype(int)
                u = (x1 + x2) // 2
                v = (y1 + y2) // 2
                (XYZ_cam, z) = self.get_point_coord(depth_m, (u, v))
                if not np.isfinite(z) or z <= 0:
                    continue
                XYZ_common = self.cam_to_common(XYZ_cam)
                name = (self.yolo.names[int(cls_id)]
                        if hasattr(self.yolo, "names") else str(int(cls_id)))
                detected_names.append(name)

                detected_objs.append({
                    'name': name,
                    'xyz': XYZ_common,
                    'confidence': c
                })

                x, y, zc = XYZ_common
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(vis, (u, v), 3, (0, 0, 255), -1)
                cv2.putText(vis, f"{name} {c:.2f} x={x:.3f} y={y:.3f} z={zc:.3f}",
                            (x1, max(0, y1-7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

        # 색상 추적 (타겟 색상이 설정된 경우만)
        direction_info = ""
        if self.target_color is not None:
            low_hsv, high_hsv = self.color_ranges[self.target_color]
            cpt = self.find_centroid(color_bgr, low_hsv, high_hsv)

            if cpt is not None:
                cx, cy = cpt

                # 색상별로 다른 색깔의 원으로 표시
                if self.target_color == 'blue':
                    cv2.circle(vis, (cx, cy), 8, (255, 0, 0), -1)
                elif self.target_color == 'green':
                    cv2.circle(vis, (cx, cy), 8, (0, 255, 0), -1)
                elif self.target_color == 'orange':
                    cv2.circle(vis, (cx, cy), 8, (0, 165, 255), -1)

                cv2.putText(vis, self.target_color, (cx-10, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

                # 방향 결정
                direction_threshold = 30
                if cx < goal_x - direction_threshold:
                    direction = "left"
                elif cx > goal_x + direction_threshold:
                    direction = "right"
                else:
                    direction = "center"

                # 토픽 발행 (left, right, center)
                msg = String()
                msg.data = direction
                self.direction_publisher.publish(msg)
                self.get_logger().info(f'Published: {direction} (detected: {self.target_color})')

                direction_info = f' | {direction.upper()} ({self.target_color})'
            else:
                direction_info = f' | Tracking {self.target_color.upper()}...'

        # YOLO 결과 발행
        if detected_names:
            unique_sorted = sorted(set(detected_names))
            msg = ",".join(unique_sorted)

            if detected_objs:
                first_obj = detected_objs[0]
                obj_msg = DetectedObject()
                obj_msg.object_name = first_obj['name']
                obj_msg.x = float(first_obj['xyz'][0])
                obj_msg.y = float(first_obj['xyz'][1])
                obj_msg.z = float(first_obj['xyz'][2])
                obj_msg.confidence = float(first_obj['confidence'])
                self.objects_publisher.publish(obj_msg)
        else:
            msg = "none"
        self.names_publisher.publish(String(data=msg))

        # 화면 중앙선 그리기
        cv2.line(vis, (goal_x,0), (goal_x,h), (255,0,0), 2)

        # 상단에 정보 표시
        info_text = f"YOLO: {msg}{direction_info}"
        cv2.putText(vis, info_text, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        # Depth 시각화
        depth_vis = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
        stack = np.hstack([vis, depth_vis])

        # MJPEG 스트림용 JPEG 인코딩
        ok, enc = cv2.imencode(".jpg", stack, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with JPEG_LOCK:
                global LATEST_JPEG
                LATEST_JPEG = enc.tobytes()

    def stop_camera(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)

    # MJPEG 서버 시작
    httpd = start_mjpeg_server(host="0.0.0.0", port=8080)
    print("RealSense+YOLO+ColorTracking MJPEG: http://<호스트IP>:8080/stream.mjpg")

    tracker = RealSenseColorTracker()

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            httpd.shutdown()
        except Exception:
            pass
        tracker.stop_camera()
        cv2.destroyAllWindows()
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
