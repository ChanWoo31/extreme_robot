import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        self.color_subscriber = self.create_subscription(String, '/color', 10)
        self.publisher = self.create_publisher(String, '/direction', 10)

        # HSV 색상 범위 정의
        self.color_ranges = {
            'blue': [(100, 50, 50), (130, 255, 255)],      # 파랑
            'green': [(40, 50, 50), (80, 255, 255)],       # 초록
            'orange': [(10, 50, 50), (25, 255, 255)]       # 주황
        }

        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.process_frame)

    def find_centroid(self, bgr, low_hsv, high_hsv):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(low_hsv), np.array(high_hsv))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not cnts:
            return None, mask

        # 가장 큰 컨투어 찾기
        c = max(cnts, key=cv2.contourArea)

        # 최소 면적 필터링
        if cv2.contourArea(c) < 500:
            return None, mask

        M = cv2.moments(c)
        if M["m00"] == 0:
            return None, mask

        return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])), mask

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]
        goal_x = w // 2  # 화면 중심 x좌표

        detected_objects = []

        # 각 색상에 대해 검사
        for color_name, (low_hsv, high_hsv) in self.color_ranges.items():
            cpt, mask = self.find_centroid(frame, low_hsv, high_hsv)

            if cpt is not None:
                cx, cy = cpt
                detected_objects.append((color_name, cx, cy))

                # 색상별로 다른 색깔의 원으로 표시
                if color_name == 'blue':
                    cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)
                elif color_name == 'green':
                    cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                elif color_name == 'orange':
                    cv2.circle(frame, (cx, cy), 8, (0, 165, 255), -1)

                cv2.putText(frame, color_name, (cx-20, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        # 가장 가까운 객체 선택 (화면 중앙에서 가장 가까운)
        if detected_objects:
            closest_obj = min(detected_objects, key=lambda obj: abs(obj[1] - goal_x))
            color_name, cx, cy = closest_obj

            # 방향 결정
            direction_threshold = 30
            if cx < goal_x - direction_threshold:
                direction = "left"
            elif cx > goal_x + direction_threshold:
                direction = "right"
            else:
                direction = "center"

            # 토픽 발행 (left, right만)
            if direction in ["left", "right"]:
                msg = String()
                msg.data = direction
                self.publisher.publish(msg)
                self.get_logger().info(f'Published: {direction} (detected: {color_name})')

            # 화면에 방향 표시
            cv2.putText(frame, f'{direction.upper()} ({color_name})', (10,30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        # 화면 중앙선 그리기
        cv2.line(frame, (goal_x,0), (goal_x,h), (255,0,0), 2)

        cv2.imshow("Color Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    color_tracker = ColorTracker()

    try:
        rclpy.spin(color_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        color_tracker.cap.release()
        cv2.destroyAllWindows()
        color_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
