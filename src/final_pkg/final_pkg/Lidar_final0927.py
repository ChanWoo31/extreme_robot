#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
from geometry_msgs.msg import Point
from sklearn.linear_model import LinearRegression
from sklearn.cluster import DBSCAN
import time

class WallFollower(Node):
    def __init__(self, offset_y=1.0):
        super().__init__('wall_follower')
        self.lidar_height = 0.19
        self.offset_y = offset_y

        self.lidar_sub = self.create_subscription(PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)
        self.yaw_pub = self.create_publisher(Float32, '/yaw_value', 10)
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_detected', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'rviz_markers', 10)

        self.yaw_flag_start_time = time.time()
        self.force_yaw_flag = False
        self.force_yaw_start_time = 0.0

    def pointcloud_callback(self, msg):
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True)])
        if len(points) == 0:
            return

        points[:,2] += self.lidar_height
        x, y, z = points[:,0], points[:,1], points[:,2]

        # ------------------ 전방 장애물 감지 ------------------
        angles = np.degrees(np.arctan2(y, x))
        forward_mask = (x > 0) & (x < 1.2) & (z <= 0.15) & (z >= 0.03) & (np.abs(angles) <= 3)
        obstacle_points = points[forward_mask]
        obstacle_detected = len(obstacle_points) > 4
        self.obstacle_pub.publish(Bool(data=obstacle_detected))

        # ------------------ 전방 벽 감지 및 Y offset 조정 ------------------
        front_mask = (np.abs(z - 0.35) < 0.05) & (np.linalg.norm(points[:, :2], axis=1) <= 1.5)
        front_angles = np.degrees(np.arctan2(y, x))
        front_mask = front_mask & (np.abs(front_angles) <= 5)
        front_wall_points = points[front_mask]
        self.offset_y = 1.5 if len(front_wall_points) >= 20 else 0.6

        # ------------------ 전체 벽 후보 ------------------
        wall_mask = (np.abs(z - 0.35) < 0.05) & (x > 0)
        wall_points = points[wall_mask]
        wall_2d = wall_points[:, :2]

        target = None
        yaw_deg = None
        right_wall_points = []

        # ------------------ 오른쪽 벽 인식 ------------------
        if len(wall_2d) > 5:
            angles_2d = np.degrees(np.arctan2(wall_2d[:,1], wall_2d[:,0]))
            # right_mask = (angles_2d <= -20) & (angles_2d >= -30)
            right_mask = (angles_2d <= -30) & (angles_2d >= -45)
            right_candidates = wall_2d[right_mask]

            if len(right_candidates) > 10:
                clustering = DBSCAN(eps=0.1, min_samples=3).fit(right_candidates)
                labels = clustering.labels_
                unique_labels = set(labels)
                max_count = 0
                main_cluster_points = np.array([])

                for lbl in unique_labels:
                    if lbl == -1:
                        continue
                    cluster_pts = right_candidates[labels == lbl]
                    if len(cluster_pts) > max_count:
                        max_count = len(cluster_pts)
                        main_cluster_points = cluster_pts

                if max_count > 10:
                    X = main_cluster_points[:,0].reshape(-1,1)
                    Y = main_cluster_points[:,1]
                    reg = LinearRegression().fit(X, Y)
                    m, b = reg.coef_[0], reg.intercept_

                    line_thresh = 0.05
                    dist_to_line = np.abs(Y - (m*X.flatten() + b))
                    mask_line = dist_to_line < line_thresh
                    right_wall_points = main_cluster_points[mask_line]

            # ------------------ 목표점 및 YAW 계산 ------------------
            if len(right_wall_points) > 0:
                x_target = np.max(right_wall_points[:,0])
                y_target = m * x_target + b + self.offset_y
                z_target = 0.35
                target = np.array([x_target, y_target, z_target])
                yaw = math.atan2(y_target, x_target)
                yaw_deg = math.degrees(yaw)
            else:
                if len(wall_2d) > 0:
                    front_mask = (angles_2d >= -30) & (angles_2d <= 30)
                    front_wall_2d = wall_2d[front_mask] if len(wall_2d[front_mask]) > 0 else wall_2d
                    dists = np.linalg.norm(front_wall_2d, axis=1)
                    idx = np.argmax(dists)
                    x_target, y_target = front_wall_2d[idx]
                    z_target = 0.35
                    target = np.array([x_target, y_target, z_target])
                    yaw = math.atan2(y_target, x_target)
                    yaw_deg = math.degrees(yaw)

        # ------------------ Force Yaw 처리 ------------------
        current_time = time.time()
        if yaw_deg is not None and -10 <= yaw_deg <= 10:
            # 정상 yaw 들어오면 force 종료
            self.force_yaw_flag = False
            self.yaw_flag_start_time = current_time
        else:
            if not hasattr(self, 'yaw_flag_start_time'):
                self.yaw_flag_start_time = current_time
            if current_time - self.yaw_flag_start_time >= 10.0:
                self.force_yaw_flag = True
                self.force_yaw_start_time = current_time

        if self.force_yaw_flag:
            if current_time - self.force_yaw_start_time <= 2.0:
                yaw_deg = 0.0
            else:
                self.force_yaw_flag = False
                self.yaw_flag_start_time = current_time

        # ------------------ YAW 퍼블리시 ------------------
        if yaw_deg is not None:
            self.yaw_pub.publish(Float32(data=float(yaw_deg)))

        # ------------------ RViz Marker ------------------
        markers = MarkerArray()

        if obstacle_detected:
            m = Marker()
            m.header.frame_id = "velodyne"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "obstacle"
            m.id = 0
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in obstacle_points]
            markers.markers.append(m)

        if len(wall_2d) > 0:
            m = Marker()
            m.header.frame_id = "velodyne"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "wall"
            m.id = 1
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.35) for p in wall_2d]
            markers.markers.append(m)

        if len(right_wall_points) > 0:
            m = Marker()
            m.header.frame_id = "velodyne"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "right_wall"
            m.id = 2
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 1.0
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.35) for p in right_wall_points]
            markers.markers.append(m)

        if target is not None:
            m = Marker()
            m.header.frame_id = "velodyne"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "target"
            m.id = 3
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(target[0])
            m.pose.position.y = float(target[1])
            m.pose.position.z = float(target[2])
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            markers.markers.append(m)

            arrow = Marker()
            arrow.header.frame_id = "velodyne"
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "yaw_arrow"
            arrow.id = 4
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.05
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            start_pt = Point(x=0.0, y=0.0, z=0.0)
            end_pt = Point(x=float(target[0]), y=float(target[1]), z=float(target[2]))
            arrow.points = [start_pt, end_pt]
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)
        print(f"Obstacle detected: {obstacle_detected}, YAW (deg): {yaw_deg}, Y offset: {self.offset_y}")


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
