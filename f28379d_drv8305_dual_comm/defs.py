from enum import IntEnum

read_variables = (
    "time",
    "motor1_pos",
    "motor1_speed",
    "motor1_torque",
    "motor2_pos",
    "motor2_speed",
    "motor2_torque",
)

_read_format = ">clffffffc"
_write_format = ">Bf"


class ControlMode(IntEnum):
    Speed_Control = 0
    Torque_Control = 1
    Position_Control_Speed = 2
    Position_Control_Direct = 3
    Bang_Bang = 4
    Impedance_Control = 5
