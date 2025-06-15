from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.motors_bus import MotorCalibration
import time
import joblib
import pandas as pd
from lerobot.common.utils.utils import enter_pressed
from pynput import keyboard
import cv2

df = pd.read_parquet("recordings/1749997013.parquet")
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

for _, row in df.iterrows():
    pos = row[["shoulder_pan", "shoulder_lift", "wrist_roll", "gripper"]].to_dict()
    img = row["image"].reshape((20, 23, 3))
    bus.sync_write(
        data_name="Goal_Position",
        values=pos,
    )
    time.sleep(0.01)

bus.disconnect()
