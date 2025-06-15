from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.motors_bus import MotorCalibration
import time
import joblib
import pandas as pd
from lerobot.common.utils.utils import enter_pressed
from pynput import keyboard
import cv2
import matplotlib.pyplot as plt

calibration = joblib.load("small_robot_calibration.joblib")

motors = [
    "shoulder_pan",
    "shoulder_lift",
    # "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

bus = FeetechMotorsBus(
    # port="/dev/tty.usbmodem59700730781",
    # port="/dev/tty.usbmodem5A460817801",
    # port="/dev/tty.usbmodem5A460851571",
    port="/dev/tty.usbmodem5A460851551",
    motors={
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
        # "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
        "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
        "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_M100_100),
    },
    calibration=calibration,
)
bus.connect()
bus.write_calibration(calibration)


def read_pos(verbose=True):
    pos = bus.sync_read(
        data_name="Present_Position",
        motors=motors,
    )
    if verbose:
        print(pos)

    return pos


def default_pos():
    bus.sync_write(
        data_name="Goal_Position",
        values={
            "shoulder_pan": 0.5520794994479132,
            "shoulder_lift": -86.56093489148581,
            "wrist_flex": 98.0688497061293,
            "wrist_roll": -53.09345068599534,
            "gripper": -95.50417469492614,
        },
    )
    time.sleep(1)
    pass


def resize_and_crop_image(image, target_size=(32, 20), crop_right_pixels=9):
    resized_image = cv2.resize(image, target_size)
    cropped_image = resized_image[:, :-crop_right_pixels, :]
    return cropped_image


time.sleep(0.2)
print("start")
bus.disable_torque(motors=motors)
video_capture = cv2.VideoCapture(0)
ret, frame = video_capture.read()
img = resize_and_crop_image(frame)
data_point = read_pos(verbose=False)
data_point["image"] = img.reshape(-1)
df = pd.DataFrame([data_point])

was_enter_pressed = False
last_time_stamp = time.time() - 1000
while not was_enter_pressed:
    if time.time() - last_time_stamp > 0.01:
        data_point = read_pos(verbose=False)
        data_point["image"] = img.reshape(-1)
        df.loc[len(df)] = data_point
        last_time_stamp = time.time()
    was_enter_pressed = enter_pressed()
file_name = f"recordings/{int(time.time())}.parquet"
print(len(df), file_name)
df.to_parquet(file_name)
print("fin")

bus.disconnect()
