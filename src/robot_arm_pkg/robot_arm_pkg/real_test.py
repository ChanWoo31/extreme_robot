import cv2

cap = cv2.VideoCapture("/dev/video10")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("video10", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
