import cv2

cap = cv2.VideoCapture(0)
if cap.isOpened():
    print("✓ 摄像头打开成功")
    ret, frame = cap.read()
    if ret:
        print("✓ 成功捕获帧")
    else:
        print("✗ 无法捕获帧")
    cap.release()
else:
    print("✗ 无法打开摄像头")

# 尝试索引 1
cap = cv2.VideoCapture(1)
if cap.isOpened():
    print("✓ 摄像头索引 1 可用")
    cap.release()