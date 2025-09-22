import cv2
import time
import numpy as np
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import pyrealsense2 as rs
from ultralytics import YOLO

# ---- ROS2 (rclpy) ----
import rclpy
from std_msgs.msg import String
from arm_interfaces.msg import DetectedObject

# ---- mjpg-streamer ----
import subprocess
import shutil

# ---------- MJPEG 서버 (RealSense+YOLO) ----------

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

# ---------- mjpg-streamer (일반 웹캠용) ----------
def start_mjpg_streamer(
    device="4",
    width=640,
    height=480,
    fps=30,
    port=8090,
    yuyv=False,
    extra_input_args=None,
    extra_output_args=None,
):
    if shutil.which("mjpg_streamer") is None:
        raise RuntimeError("mjpg_streamer 실행 파일을 찾을 수 없습니다. 설치를 확인하세요.")

    input_plugin = "input_uvc.so"
    output_plugin = "output_http.so"

    input_args = [
        input_plugin,
        "-d", device,
        "-r", f"{width}x{height}",
        "-f", str(fps),
    ]
    if yuyv:
        input_args.append("-y")

    if extra_input_args:
        input_args.extend(extra_input_args)

    output_args = [
        output_plugin,
        "-p", str(port),
        "-w", "/usr/local/share/mjpg-streamer/www"
    ]
    if extra_output_args:
        output_args.extend(extra_output_args)

    cmd = ["mjpg_streamer", "-i", " ".join(input_args), "-o", " ".join(output_args)]
    print("[mjpg-streamer] CMD:", " ".join(cmd))

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(0.8)
    return proc

def stop_mjpg_streamer(proc: subprocess.Popen):
    if proc is None:
        return
    try:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
    except Exception:
        pass

# ---------- RealSense + YOLO ----------

class RealSense:
    def __init__(self):
        self.TIMEOUT_MS = 150000
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

def main():
    # ---- ROS2 노드 & 퍼블리셔 초기화 ----
    rclpy.init()
    node = rclpy.create_node('yolo_realsense_mjpeg')
    pub_names = node.create_publisher(String, '/detected_object_names', 10)
    pub_objects = node.create_publisher(DetectedObject, '/detected_objects', 10)

    # ---- RealSense + YOLO (파이썬 내장 MJPEG 서버, 8080) ----
    rsys = RealSense()
    httpd = start_mjpeg_server(host="0.0.0.0", port=8080)
    node.get_logger().info("RealSense+YOLO MJPEG: http://<호스트IP>:8080/stream.mjpg")

    # ---- 일반 웹캠을 mjpg-streamer로 송출 (8090) ----
    try:
        webcam_proc = start_mjpg_streamer(
            device="/dev/video4",
            width=640,
            height=480,
            fps=30,
            port=8090,
            yuyv=False,
        )
        node.get_logger().info("Webcam via mjpg-streamer: http://<호스트IP>:8090/?action=stream")
    except Exception as e:
        webcam_proc = None
        node.get_logger().warn(f"mjpg-streamer 시작 실패: {e}")

    try:
        while rclpy.ok():
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
                    (XYZ_cam, z) = rsys.get_point_coord(depth_m, (u, v))
                    if not np.isfinite(z) or z <= 0:
                        continue
                    XYZ_common = rsys.cam_to_common(XYZ_cam)
                    name = (rsys.yolo.names[int(cls_id)]
                            if hasattr(rsys.yolo, "names") else str(int(cls_id)))
                    detected_names.append(name)

                    # detected_objs 리스트에 객체 정보 추가
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

            if detected_names:
                unique_sorted = sorted(set(detected_names))
                msg = ",".join(unique_sorted)

                # DetectedObject 메시지도 발행 (첫 번째 검출된 객체)
                if detected_objs:
                    first_obj = detected_objs[0]  # 첫 번째 검출 객체
                    obj_msg = DetectedObject()
                    obj_msg.object_name = first_obj['name']
                    obj_msg.x = float(first_obj['xyz'][0])
                    obj_msg.y = float(first_obj['xyz'][1])
                    obj_msg.z = float(first_obj['xyz'][2])
                    obj_msg.confidence = float(first_obj['confidence'])
                    pub_objects.publish(obj_msg)

                    node.get_logger().info(f"Published DetectedObject: {obj_msg.object_name} at ({obj_msg.x:.3f}, {obj_msg.y:.3f}, {obj_msg.z:.3f})")
            else:
                msg = "none"
            pub_names.publish(String(data=msg))

            depth_vis = np.asanyarray(rsys.colorizer.colorize(depth_frame).get_data())
            stack = np.hstack([vis, depth_vis])

            ok, enc = cv2.imencode(".jpg", stack, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with JPEG_LOCK:
                    global LATEST_JPEG
                    LATEST_JPEG = enc.tobytes()

            # 필요하면 FPS 제어
            # time.sleep(1.0/rsys.FPS)

    except KeyboardInterrupt:
        pass
    finally:
        stop_mjpg_streamer(webcam_proc)
        try:
            httpd.shutdown()
        except Exception:
            pass
        rsys.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
