import os
import json
from collections.abc import Callable
from typing import Dict, Tuple
from pynput.keyboard import Listener

from so101_ros2.lerobot.common.motors import FeetechMotorsBus, Motor, MotorNormMode, MotorCalibration, OperatingMode
from so101_ros2.lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from so101_ros2.lerobot.device_base import Device

# motor limit written in real device (normalized to related range)
SO101_FOLLOWER_MOTOR_LIMITS = {
    'shoulder_pan': (-100.0, 100.0),
    'shoulder_lift': (-100.0, 100.0),
    'elbow_flex': (-100.0, 100.0),
    'wrist_flex': (-100.0, 100.0),
    'wrist_roll': (-100.0, 100.0),
    'gripper': (0.0, 100.0),
}

class SO101(Device):
    """A SO101 Leader device for SE(3) control.
    """

    def __init__(self, port: str = '/dev/ttyACM0', name: str = 'so101_leader', recalibrate: bool = False):
        super().__init__()
        self.port = port
        self.name = name
        # calibration
        self.calibration_path = os.path.join(os.path.dirname(__file__), ".cache", f"{self.name}.json")
        if not os.path.exists(self.calibration_path) or recalibrate:
            self.calibrate()
        calibration = self._load_calibration()

        self._bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=calibration,
        )
        self._motor_limits = SO101_FOLLOWER_MOTOR_LIMITS

        # connect
        # self.connect()

        # some flags and callbacks
        self._started = False
        self._reset_state = False
        self._additional_callbacks = {}

    def __str__(self) -> str:
        """Returns: A string containing the information of so101 leader."""
        msg = "SO101-Leader device for SE(3) control.\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tMove SO101-Leader to control SO101-Follower\n"
        msg += "\tIf SO101-Follower can't synchronize with SO101-Leader, please add --recalibrate and rerun to recalibrate SO101-Leader.\n"
        return msg

    def get_device_state(self):
        return self._bus.sync_read("Present_Position")

    def add_callback(self, key: str, func: Callable):
        self._additional_callbacks[key] = func

    @property
    def motor_limits(self) -> Dict[str, Tuple[float, float]]:
        return self._motor_limits

    @property
    def is_connected(self) -> bool:
        return self._bus.is_connected

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError("SO101-Leader is not connected.")
        self._bus.disconnect()
        print("SO101-Leader disconnected.")

    def connect(self):
        if self.is_connected:
            raise DeviceAlreadyConnectedError("SO101-Leader is already connected.")
        self._bus.connect()
        self.configure()
        print(f"SO101 Arm initialized with: Port={self.port}, Name={self.name}")

    def configure(self) -> None:
        self._bus.disable_torque()
        self._bus.configure_motors()
        for motor in self._bus.motors:
            self._bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def calibrate(self):
        self._bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
        )
        self.connect()

        print("\n Running calibration of SO101-Leader")
        self._bus.disable_torque()
        for motor in self._bus.motors:
            self._bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input("Move SO101-Leader to the middle of its range of motion and press ENTER...")
        homing_offset = self._bus.set_half_turn_homings()
        print("Move all joints sequentially through their entire ranges of motion.")
        print("Recording positions. Press ENTER to stop...")
        range_mins, range_maxes = self._bus.record_ranges_of_motion()

        calibration = {}
        for motor, m in self._bus.motors.items():
            calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offset[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        self._bus.write_calibration(calibration)
        self._save_calibration(calibration)
        print(f"Calibration saved to {self.calibration_path}")

        self.disconnect()

    def _load_calibration(self) -> Dict[str, MotorCalibration]:
        with open(self.calibration_path, "r") as f:
            json_data = json.load(f)
        calibration = {}
        for motor_name, motor_data in json_data.items():
            calibration[motor_name] = MotorCalibration(
                id=int(motor_data["id"]),
                drive_mode=int(motor_data["drive_mode"]),
                homing_offset=int(motor_data["homing_offset"]),
                range_min=int(motor_data["range_min"]),
                range_max=int(motor_data["range_max"]),
            )
        return calibration

    def _save_calibration(self, calibration: Dict[str, MotorCalibration]):
        save_calibration = {k: {
            "id": v.id,
            "drive_mode": v.drive_mode,
            "homing_offset": v.homing_offset,
            "range_min": v.range_min,
            "range_max": v.range_max,
        } for k, v in calibration.items()}
        if not os.path.exists(os.path.dirname(self.calibration_path)):
            os.makedirs(os.path.dirname(self.calibration_path))
        with open(self.calibration_path, 'w') as f:
            json.dump(save_calibration, f, indent=4)
