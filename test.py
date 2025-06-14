from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.motors_bus import MotorCalibration
import time
import joblib
import pandas as pd
from lerobot.common.utils.utils import enter_pressed
from pynput import keyboard

w_pressed = False
s_pressed = False
a_pressed = False
d_pressed = False
q_pressed = False
e_pressed = False
zero_pressed = False
one_pressed = False
two_pressed = False
y_pressed = False
x_pressed = False


def on_press(key):
    global w_pressed
    global s_pressed
    global a_pressed
    global d_pressed
    global q_pressed
    global e_pressed
    global one_pressed
    global two_pressed
    global zero_pressed
    global y_pressed
    global x_pressed
    try:
        print("Key pressed:", key)
        if key.char == "w":
            w_pressed = True
        if key.char == "s":
            s_pressed = True
        if key.char == "a":
            a_pressed = True
        if key.char == "d":
            d_pressed = True
        if key.char == "q":
            q_pressed = True
        if key.char == "e":
            e_pressed = True
        if key.char == "1":
            one_pressed = True
        if key.char == "2":
            two_pressed = True
        if key.char == "0":
            zero_pressed = True
        if key.char == "y":
            y_pressed = True
        if key.char == "x":
            x_pressed = True
    except AttributeError:
        pass


def on_release(key):
    global w_pressed
    global s_pressed
    global a_pressed
    global d_pressed
    global q_pressed
    global e_pressed
    global one_pressed
    global two_pressed
    global zero_pressed
    global y_pressed
    global x_pressed
    try:
        if key.char == "w":
            w_pressed = False
        if key.char == "s":
            s_pressed = False
        if key.char == "a":
            a_pressed = False
        if key.char == "d":
            d_pressed = False
        if key.char == "q":
            q_pressed = False
        if key.char == "e":
            e_pressed = False
        if key.char == "1":
            one_pressed = False
        if key.char == "2":
            two_pressed = False
        if key.char == "0":
            zero_pressed = False
        if key.char == "y":
            y_pressed = False
        if key.char == "x":
            x_pressed = False
    except AttributeError:
        pass


listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

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
    port="/dev/tty.usbmodem59700730781",
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


def calibrate():
    input(f"Move all motors to the middle of its range of motion and press ENTER....")
    homing_offsets = bus.set_half_turn_homings()

    range_mins, range_maxes = bus.record_ranges_of_motion(
        motors=motors,
        display_values=True,
    )

    calibration = {}
    for motor, m in bus.motors.items():
        calibration[motor] = MotorCalibration(
            id=m.id,
            drive_mode=0,
            homing_offset=homing_offsets[motor],
            range_min=range_mins[motor],
            range_max=range_maxes[motor],
        )

    bus.write_calibration(calibration)

    print("calibration", calibration)

    joblib.dump(calibration, "small_robot_calibration.joblib")


def read_pos(verbose=True):
    pos = bus.sync_read(
        data_name="Present_Position",
        motors=motors,
    )
    if verbose:
        print(pos)

    return pos


def set_pos(pos):

    for motor, value in pos.items():
        pos[motor] = min(
            max(value, -99.0),
            99.0,
        )

    bus.sync_write(
        data_name="Goal_Position",
        values=pos,
    )


def default_pos():
    bus.sync_write(
        data_name="Goal_Position",
        values={
            "shoulder_pan": 0.926298832057995,
            "shoulder_lift": -94.03611927761445,
            "wrist_flex": 98.57740585774059,
            "wrist_roll": 22.0,
            "gripper": -96.03960396039604,
        },
    )
    time.sleep(1)
    pass


def record(file_name="record.parquet"):
    input("Press ENTER to start recording...")
    bus.disable_torque(motors=motors)

    print("Recording positions. Press ENTER to stop.")

    pos = read_pos()
    df = pd.DataFrame([pos])

    was_enter_pressed = False
    last_time_stamp = time.time() - 1000
    while not was_enter_pressed:
        if time.time() - last_time_stamp > 0.01:
            df.loc[len(df)] = read_pos()
            print("Recorded position:", pos)
            last_time_stamp = time.time()
        was_enter_pressed = enter_pressed()
    print(len(df))
    df.to_parquet(file_name)


def play_record(file_name="record.parquet"):
    df = pd.read_parquet(file_name)
    print("Playing recorded positions...")

    for i, row in df.iterrows():
        bus.sync_write(
            data_name="Goal_Position",
            values=row.to_dict(),
        )
        time.sleep(0.01)


def control_with_keyboard():
    print("Control the robot with keyboard w, a, s, d keys.")

    move_dict = {"shoulder_lift": 0}
    last_time_stamp = time.time() - 1000
    current_pos = read_pos(verbose=False)
    while True:
        if time.time() - last_time_stamp > 0.01:

            for motor, value in move_dict.items():
                current_pos[motor] += value
            set_pos(current_pos)
            last_time_stamp = time.time()

        if w_pressed:
            move_dict["shoulder_lift"] = 0.5
        elif s_pressed:
            move_dict["shoulder_lift"] = -0.5
        else:
            move_dict["shoulder_lift"] = 0
        if a_pressed:
            move_dict["shoulder_pan"] = -0.5
        elif d_pressed:
            move_dict["shoulder_pan"] = 0.5
        else:
            move_dict["shoulder_pan"] = 0
        if one_pressed:
            move_dict["gripper"] = -0.5
        elif two_pressed:
            move_dict["gripper"] = 0.5
        else:
            move_dict["gripper"] = 0
        if q_pressed:
            move_dict["wrist_flex"] = -0.5
        elif e_pressed:
            move_dict["wrist_flex"] = 0.5
        else:
            move_dict["wrist_flex"] = 0
        if y_pressed:
            move_dict["wrist_roll"] = -0.5
        elif x_pressed:
            move_dict["wrist_roll"] = 0.5
        else:
            move_dict["wrist_roll"] = 0
        if zero_pressed:
            break


# calibrate()


# read_pos()
default_pos()
# play_record()
# record()
control_with_keyboard()

# # default_pos()
set_pos(
    {
        "shoulder_pan": 0.926298832057995,
        "shoulder_lift": -94.03611927761445,
        "wrist_flex": 50.57740585774059,
        "gripper": -96.03960396039604,
    }
)
time.sleep(0.35)
set_pos(
    {
        "shoulder_pan": 0.926298832057995,
        "shoulder_lift": -94.03611927761445,
        "wrist_flex": 0.0,
        "gripper": 90.0,
    }
)
time.sleep(0.65)
default_pos()
bus.disconnect()

# ToDo make this as a class and integrate it into the lerobot.common.motors package!
