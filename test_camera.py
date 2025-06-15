import cv2


video_capture = cv2.VideoCapture(0)

ret, frame = video_capture.read()
while ret:
    cv2.imshow("Camera Feed", frame)
    if cv2.waitKey(1) == ord("q"):
        break
    ret, frame = video_capture.read()
video_capture.release()
