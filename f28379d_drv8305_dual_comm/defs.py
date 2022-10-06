from enum import IntEnum


class ControlMode(IntEnum):
    Speed_Control = 0
    Torque_Control = 1
    Position_Control_Speed = 2
    Position_Control_Direct = 3
    Bang_Bang = 4
    Impedance_Control = 5
