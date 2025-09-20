import cv2
import numpy as np

# 간단한 테스트 이미지 생성
test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

print("OpenCV 창 테스트 시작...")
cv2.namedWindow("Test Window", cv2.WINDOW_AUTOSIZE)
cv2.imshow("Test Window", test_img)
print("이미지 표시됨. 아무 키나 누르세요...")
cv2.waitKey(0)
cv2.destroyAllWindows()
print("테스트 완료")