import cv2
import time
import numpy as np
import pyrealsense2 as rs

# ---------- 설정 ----------

class RealSense:
    def __init__(self):
        self.WIDTH, self.HEIGHT, self.FPS = 640, 480, 30
        self.USE_FILTERS = True  # spatial/temporal/hole-filling 필터 사용 (노이즈/홀 보정)
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

        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()

    def build_filters(self):
        spatial = rs.spatial_filter()       # edge-preserving spatial smoothing
        spatial.set_option(rs.option.holes_fill, 2)
        temporal = rs.temporal_filter()     # temporal smoothing
        hole_filling = rs.hole_filling_filter(1)  # 0:none, 1:nearest, 2:farther
        self.filters = [spatial, temporal, hole_filling]

    def apply_filters(self, depth_frame):
        spatial, temporal, hole_filling = self.filters
        f = depth_frame
        f = spatial.process(f)
        f = temporal.process(f)
        f = hole_filling.process(f)
        return f

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
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
                uu, vv = np.meshgrid(u, v)  # shape (H, W)
                xx_factor = (uu - cx) / fx  # X = xx_factor * Z
                yy_factor = (vv - cy) / fy  # Y = yy_factor * Z

                return intrinsics, (xx_factor, yy_factor)
        except:
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def get_point_coord(self, depth_img, point):
        u0, v0 = max(0, point[0] - 1), max(0, point[1] - 1)
        u1, v1 = min(self.WIDTH, point[0] + 2), min(self.HEIGHT, point[1] + 2)

        crop = depth_img[v0:v1, u0:u1]
        valid = crop[crop > 0]
        if valid.size > 0:
            z = np.median(valid)
        else:
            z = 0

        X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, point, z)

        return np.array([X, Y, Z])



def main():
    realsense = RealSense()

    try:
        while True:
            color_frame, depth_frame = realsense.get_frames()
            
            if not depth_frame or not color_frame:
                continue

            if realsense.filters is not None:
                depth_frame = realsense.apply_filters(depth_frame, realsense.filters)

            depth_raw = np.asanyarray(depth_frame.get_data())  # uint16, 단위: depth ticks
            color_bgr = np.asanyarray(color_frame.get_data())

            # 깊이(m)로 변환
            depth_m = depth_raw.astype(np.float32) * realsense.depth_scale
            # invalid(0) → NaN
            invalid_mask = depth_raw == 0
            depth_m[invalid_mask] = np.nan

            # 전체 XYZ 맵 계산 (카메라 좌표계, m)
            # X: 오른쪽(+), Y: 아래(+), Z: 전방(+)
            Z = depth_m
            X = realsense.xx_factor * Z
            Y = realsense.yy_factor * Z

            # 시각화용 컬러 depth
            depth_vis = np.asanyarray(realsense.colorizer.colorize(depth_frame).get_data())

            vis = np.hstack([color_bgr, depth_vis])
            cv2.putText(vis, "Press 'q' to quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow(realsense.WINDOW_NAME, vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), 27):  # q or ESC
                break

            print(realsense.get_point_coord(depth_m, (320, 240)))

    finally:
        realsense.stop()

if __name__ == "__main__":
    main()