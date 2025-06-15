from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.motors_bus import MotorCalibration
import time
import joblib
import pandas as pd
from lerobot.common.utils.utils import enter_pressed
from pynput import keyboard
import cv2

video_capture = cv2.VideoCapture(0)

ret, frame = video_capture.read()
user_input = ""
while ret:
    cv2.imshow("Camera Feed", frame)
    if cv2.waitKey(1) == ord("q") or user_input.strip() == "q":
        cv2.imwrite(f"camera_data/{int(time.time())}.jpg", frame)
        break
    user_input = input("Press Enter to continue...")
    ret, frame = video_capture.read()
video_capture.release()
