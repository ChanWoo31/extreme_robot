#!/usr/bin/env python3
import cv2
import numpy as np
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

# ---- ROS2 ----
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from arm_interfaces.msg import DetectedObject

# ---------- MJPEG 서버 ----------
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

# ---------- 카메라 그랩버 ----------
class CameraGrabber:
    def __init__(self, device=0, width=640, height=480, fps=30, name="cam"):
        self.dev = device
        self.w, self.h, self.fps = width, height, fps
        self.name = name
        self.cap = None
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def open(self):
        print(f"[{self.name}] Trying to open /dev/video{self.dev}")
        cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # 버퍼 크기를 최소화하여 지연 감소
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print(f"[{self.name}] Failed to open /dev/video{self.dev}")
            return False
        print(f"[{self.name}] Successfully opened /dev/video{self.dev}")
        self.cap = cap
        return True

    def _loop(self):
        t_frame = 1.0 / max(1, self.fps)
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                # 재시도
                self.open()
                time.sleep(0.5)
                continue
            ok, frm = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = frm
            else:
                print(f"[{self.name}] Frame read failed, reopening...")
                # 드롭 시 재오픈
                self.cap.release()
                self.cap = None
            time.sleep(t_frame * 0.2)

    def start(self):
        self.running = True
        if self.cap is None:
            self.open()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def get(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def flush_buffer(self):
        """카메라 버퍼를 비워서 최신 프레임 확보"""
        if self.cap is None or not self.cap.isOpened():
            return
        # 버퍼에 있는 오래된 프레임들을 버리고 최신 프레임 읽기
        for _ in range(5):  # 5프레임 정도 버퍼 비우기
            self.cap.read()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
        self.cap = None

# ---------- 색 추적 노드 ----------
class DualCamColorTracker(Node):
    def __init__(self,
                 dev0=0, dev1=2,
                 width=640, height=480, fps=30):
        super().__init__('dualcam_color_tracker')

        # ROS2 I/O
        self.color_sub = self.create_subscription(String, '/target_color', self.color_callback, 10)
        self.direction_pub = self.create_publisher(String, '/direction', 10)
        self.names_pub = self.create_publisher(String, '/detected_object_names', 10)
        self.objects_pub = self.create_publisher(DetectedObject, '/detected_objects', 10)

        # HSV 범위
        self.color_ranges = {
            'blue':   [(100, 50, 50), (130, 255, 255)],
            'green':  [(40, 50, 50),  (80, 255, 255)],
            'orange': [(10, 50, 50),  (25, 255, 255)],
        }
        self.target_color = None

        # 카메라 두 대
        self.cam0 = CameraGrabber(dev0, width, height, fps, name="cam0")
        self.cam1 = CameraGrabber(dev1, width, height, fps, name="cam1")
        self.cam0.start()
        self.cam1.start()

        self.W, self.H = width, height

        # YOLO 초기화
        self.yolo = YOLO('/home/han/extreme_robot_ws/src/robot_arm_pkg/best.pt')

        self.timer = self.create_timer(1.0/fps, self.process_once)

    def color_callback(self, msg: String):
        c = msg.data.lower().strip()
        if c == "" or c == "stop":
            self.target_color = None
            self.get_logger().info('Color tracking stopped')
        elif c in self.color_ranges:
            self.target_color = c
            self.get_logger().info(f'Target color set to: {c}')
        else:
            self.get_logger().warn('Unknown color. Use: blue, green, orange, or empty to stop')

    @staticmethod
    def find_centroid(bgr, low_hsv, high_hsv, min_area=500):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(low_hsv), np.array(high_hsv))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, mask
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < min_area:
            return None, mask
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None, mask
        return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])), mask

    def process_once(self):
        frm0 = self.cam0.get()
        frm1 = self.cam1.get()

        if frm0 is None and frm1 is None:
            return

        # 표시용 프레임 준비
        vis0 = frm0.copy() if frm0 is not None else np.zeros((480, 640, 3), np.uint8)
        vis1 = frm1.copy() if frm1 is not None else np.zeros((480, 640, 3), np.uint8)

        # ---- 실제 크기 기준으로 처리 ----
        h0, w0 = vis0.shape[:2]
        h1, w1 = vis1.shape[:2]

        # YOLO 객체 감지 (cam0 기준)
        detected_names = []
        detected_objs = []

        if frm0 is not None:
            result = self.yolo.predict(source=frm0, verbose=False, conf=0.7)[0]

            if result.boxes is not None and len(result.boxes) > 0:
                xyxy = result.boxes.xyxy.cpu().numpy()
                cls = result.boxes.cls.cpu().numpy()
                conf = result.boxes.conf.cpu().numpy()

                for box, cls_id, c in zip(xyxy, cls, conf):
                    x1, y1, x2, y2 = box.astype(int)
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2

                    name = (self.yolo.names[int(cls_id)]
                           if hasattr(self.yolo, "names") else str(int(cls_id)))
                    detected_names.append(name)

                    detected_objs.append({
                        'name': name,
                        'center': (u, v),
                        'confidence': c,
                        'box': (x1, y1, x2, y2)
                    })

                    # 바운딩 박스와 라벨 그리기
                    cv2.rectangle(vis0, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(vis0, (u, v), 3, (0, 0, 255), -1)
                    cv2.putText(vis0, f"{name} {c:.2f}", (x1, max(0, y1-7)),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # 기준 카메라: cam0
        goal_x = w0 // 2  # 실제 가로폭 기준
        direction_info = "idle"
        direction = None

        if self.target_color is not None and frm0 is not None:
            low, high = self.color_ranges[self.target_color]
            cpt, mask = self.find_centroid(vis0, low, high)

            if cpt is not None:
                cx, cy = cpt
                if self.target_color == 'blue':
                    color = (255, 0, 0)
                elif self.target_color == 'green':
                    color = (0, 255, 0)
                else:
                    color = (0, 165, 255)
                cv2.circle(vis0, (cx, cy), 8, color, -1)
                cv2.putText(vis0, self.target_color, (cx-10, cy-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

                thr = max(4, int(0.05 * w0))  # 가로폭 대비 임계값
                if cx < goal_x - thr:
                    direction = "left"
                elif cx > goal_x + thr:
                    direction = "right"
                else:
                    direction = "center"

                self.direction_pub.publish(String(data=direction))
                direction_info = f'{direction} ({self.target_color})'
            else:
                direction_info = f'tracking {self.target_color}...'

        # 중앙선 (리사이즈 전 기준으로 그리고 함께 스케일됨)
        cv2.line(vis0, (goal_x, 0), (goal_x, h0), (255, 0, 0), 2)

        # YOLO 결과 퍼블리시
        if detected_names:
            unique_sorted = sorted(set(detected_names))
            msg = ",".join(unique_sorted)

            if detected_objs:
                first_obj = detected_objs[0]
                obj_msg = DetectedObject()
                obj_msg.object_name = first_obj['name']
                obj_msg.confidence = float(first_obj['confidence'])
                cx, cy = first_obj['center']
                obj_msg.pixel_x = float(cx)
                obj_msg.pixel_y = float(cy)

                # 카메라 중앙점과의 거리 계산 (실제 크기 기준)
                obj_msg.distance_from_center_x = float(cx - (w0 // 2))
                obj_msg.distance_from_center_y = float(-(cy - (h0 // 2)))  # y축 반전

                self.objects_pub.publish(obj_msg)
        else:
            msg = "none"

        self.names_pub.publish(String(data=msg))

        # 상태 텍스트
        info = f"DualCam+YOLO | detected={msg} | target={self.target_color or 'none'} | {direction_info}"
        cv2.putText(vis0, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        # ---- 동일 높이로만 리사이즈하여 가로비 유지 ----
        target_h = 480
        scale0 = target_h / h0
        scale1 = target_h / h1
        vis0 = cv2.resize(vis0, (int(w0 * scale0), target_h))
        vis1 = cv2.resize(vis1, (int(w1 * scale1), target_h))

        stack = np.hstack([vis0, vis1])

        # MJPEG용 JPEG 인코딩
        ok, enc = cv2.imencode(".jpg", stack, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with JPEG_LOCK:
                global LATEST_JPEG
                LATEST_JPEG = enc.tobytes()

    def stop(self):
        self.cam0.stop()
        self.cam1.stop()

# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)

    # MJPEG 서버 시작
    httpd = start_mjpeg_server(host="0.0.0.0", port=8080)
    print("Dual USB Cameras MJPEG: http://<호스트IP>:8080/stream.mjpg")

    # 필요 시 디바이스 인덱스 변경
    tracker = DualCamColorTracker(dev0=4, dev1=6, width=400, height=300, fps=15)

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            httpd.shutdown()
        except Exception:
            pass
        tracker.stop()
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
