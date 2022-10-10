import logging
import re
import struct
import threading
from queue import Queue

import serial

from f28379d_drv8305_dual_comm.defs import (
    data_variables,
    sampling_rate,
    _read_format,
    _write_format,
    ControlMode,
)
from f28379d_drv8305_dual_comm.funcs import crc_calculator, find_port


class Controller:
    """
    Communicate using serial with a f28379d used to control two motors through drv8305.
    """

    def __init__(self, port=None, device_name=None, baud=115200):

        if port is None:
            self.device_name = "XDS100" if device_name is None else device_name
            port_found = find_port(self.device_name)
            if port_found is not None:
                self.port = port_found
            else:
                raise ValueError("No valid port found.")
        else:
            self.port = port

        self.baud = baud

        self.read_queue = Queue(sampling_rate)
        """
        Data from motors are put into a read queue by default.
        To get data from motors, either read from this queue or set a callback function.
        """

        self._thread = None
        self._write_to_queue = True
        self._on_motor_measurement_cb = None
        self._stop_serial = False

        self._header = data_variables
        self._read_struct = struct.Struct(_read_format)
        self._write_struct = struct.Struct(_write_format)
        self._checksum_struct = struct.Struct(">H")

        self._selected_control_mode = ControlMode.Speed_Control

        self._write_queue = Queue()
        self._start_serial()

    def set_control_mode(self, motor_no: int, mode: str):
        """
        Set motor control mode.

        :param motor_no: 1 or 2
        :param mode: Choose between speed control, position control (speed), position control (direct), bang bang, and impedance control modes
        """

        if isinstance(mode, str):
            mode = mode.lower()
            subs = [
                (r"_|-", " "),
                (r"\(|\)|\[|\]", ""),
            ]
            for old, new in subs:
                mode = re.sub(old, new, mode)

            match mode:
                case "speed control" | "speed":
                    self._selected_control_mode = ControlMode.Speed_Control
                case "torque control" | "torque" | "constant torque" | "hold torque":
                    self._selected_control_mode = ControlMode.Torque_Control
                case "position control speed" | "position speed" | "pos speed" | "position speed control" | "pos speed control":
                    self._selected_control_mode = ControlMode.Position_Control_Speed
                case "position control direct" | "position direct" | "position direct control" | "pos control direct" | "pos direct" | "pos direct control":
                    self._selected_control_mode = ControlMode.Position_Control_Direct
                case "bang bang control" | "bang bang" | "bang":
                    self._selected_control_mode = ControlMode.Bang_Bang
                case "impedance control" | "impedance":
                    self._selected_control_mode = ControlMode.Impedance_Control
                case _:
                    logging.error("invalid control mode")

        elif isinstance(mode, (ControlMode, int)):
            self._selected_control_mode = mode

        else:
            logging.error("invalid control mode")

        self._write_queue.put((b"\x01", motor_no, float(self._selected_control_mode)))

    def set_max_torque(self, motor_no: int, torque: float):
        """
        Set maximum torque in N m.
        Used in position control (speed), position control (direct), bang bang, and impedance control modes.

        :param motor_no: 1 or 2
        :param torque: Torque in N m (max torque = 4.3 N m)
        """
        self._write_queue.put((b"\x02", motor_no, torque))

    def set_speed(self, motor_no: int, speed: float):
        """
        Set target speed.
        Used in speed control mode.

        :param motor_no: 1 or 2
        :param speed: Rotational speed in rad/s (max speed = 600 rad/s)
        """
        self._write_queue.put((b"\x03", motor_no, speed))

    def set_pos(self, motor_no: int, pos: float):
        """
        Set target pos.
        Used in position control (speed), position control (direct), bang bang, and impedance control modes.

        :param motor_no: 1 or 2
        :param pos: Target angle in radians
        """
        self._write_queue.put((b"\x04", motor_no, pos))

    def set_speed_p(self, motor_no: int, gain: float):
        """
        Set P gain for speed control.
        Used in speed control and position control (speed) modes.

        :param motor_no: 1 or 2
        :param gain: Proportional gain
        """
        self._write_queue.put((b"\x11", motor_no, gain))

    def set_speed_i(self, motor_no: int, gain: float):
        """
        Set I gain for speed control.
        Used in speed control and position control (speed) modes.

        :param motor_no: 1 or 2
        :param gain: Integral gain
        """
        self._write_queue.put((b"\x12", motor_no, gain))

    def set_speed_d(self, motor_no: int, gain: float):
        """
        Set D gain for speed control.
        Used in speed control and position control (speed) modes.

        :param motor_no: 1 or 2
        :param gain: Derivative gain
        """
        self._write_queue.put((b"\x13", motor_no, gain))

    def set_hold_torque(self, motor_no: int, torque: float):
        """
        Set constant torque.
        Used in torque control mode.

        :param motor_no: 1 or 2
        :param torque: Torque in N m (max torque = 4.3 N m)
        """
        self._write_queue.put((b"\x21", motor_no, torque))

    def set_pos_p(self, motor_no: int, gain: float):
        """
        Set P gain for position control (speed).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param gain: Proportional gain
        """
        self._write_queue.put((b"\x31", motor_no, gain))

    def set_pos_i(self, motor_no: int, gain: float):
        """
        Set I gain for position control (speed).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param gain: Integral gain
        """
        self._write_queue.put((b"\x32", motor_no, gain))

    def set_pos_d(self, motor_no: int, gain: float):
        """
        Set D gain for position control (speed).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param gain: Derivative gain
        """
        self._write_queue.put((b"\x33", motor_no, gain))

    def set_max_speed(self, motor_no: int, speed: float):
        """
        Set maximum target speed for position control.
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param speed: Rotational speed in rad/s (max speed = 600 rad/s)
        """
        self._write_queue.put((b"\x34", motor_no, speed))

    def set_pos_p_direct(self, motor_no: int, gain: float):
        """
        Set P gain for position control (direct).
        Used in position control (direct) mode.

        :param motor_no: 1 or 2
        :param gain: Proportional gain
        """
        self._write_queue.put((b"\x41", motor_no, gain))

    def set_pos_i_direct(self, motor_no: int, gain: float):
        """
        Set I gain for position control (direct).
        Used in position control (direct) mode.

        :param motor_no: 1 or 2
        :param gain: Integral gain
        """
        self._write_queue.put((b"\x42", motor_no, gain))

    def set_pos_d_direct(self, motor_no: int, gain: float):
        """
        Set D gain for position control (direct).
        Used in position control (direct) mode.

        :param motor_no: 1 or 2
        :param gain: Derivative gain
        """
        self._write_queue.put((b"\x43", motor_no, gain))

    def set_bang_bang_torque(self, motor_no: int, torque: float):
        """
        Set constant torque in bang-bang control.
        Used in bang-bang control mode.

        :param motor_no: 1 or 2
        :param torque: Torque in N m (max torque = 4.3 N m)
        """
        self._write_queue.put((b"\x51", motor_no, torque))

    def set_bang_bang_deadband(self, motor_no: int, deadband: float):
        """
        Set deadband in radians in bang-bang control.
        Used in bang-bang control mode.

        :param motor_no: 1 or 2
        :param deadband: Deadband in radians
        """
        self._write_queue.put((b"\x52", motor_no, deadband))

    def set_inertia(self, motor_no: int, inertia: float):
        """
        Set rotational inertia.
        Used in impedance control mode.

        :param motor_no: 1 or 2
        :param inertia: Inertia
        """
        self._write_queue.put((b"\x61", motor_no, inertia))

    def set_damping(self, motor_no: int, damping: float):
        """
        Set rotational damping.
        Used in impedance control mode.

        :param motor_no: 1 or 2
        :param damping: Damping
        """
        self._write_queue.put((b"\x62", motor_no, damping))

    def set_stiffness(self, motor_no: int, stiffness: float):
        """
        Set rotational stiffness.
        Used in impedance control mode.

        :param motor_no: 1 or 2
        :param stiffness: Stiffness
        """
        self._write_queue.put((b"\x63", motor_no, stiffness))

    def set_zero_angle(self, motor_no: int):
        """
        Set zero angle.
        Used in position control (speed), position control (direct), bang bang, and impedance control modes.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF0", motor_no, 1))

    def realign_motor(self, motor_no: int):
        """
        Realign motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF2", motor_no, 1))

    def enable_motor(self, motor_no: int):
        """
        Enable motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF3", motor_no, 1))

    def disable_motor(self, motor_no: int):
        """
        Disable motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF3", motor_no, 0))

    def _start_serial(self):
        self._thread = threading.Thread(target=self._read_serial_data, daemon=True)
        self._thread.start()

    def _read_serial_data(self):
        ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=1)
        while not self._stop_serial:
            raw_data = ser.read(self._read_struct.size)
            if not raw_data:
                continue
            unpacked_data = self._read_struct.unpack(raw_data)
            if unpacked_data[-1] != b"\n":
                _ = ser.readline()
            elif self._write_to_queue:
                self.read_queue.put(dict(zip(self._header, unpacked_data[1:-1])))
            elif callable(self._on_motor_measurement_cb):
                self._on_motor_measurement_cb(
                    dict(zip(self._header, unpacked_data[1:-1]))
                )
            if not self._write_queue.empty():
                identifier, motor_no, val = self._write_queue.get()
                msg_packed = self._write_struct.pack(motor_no, val)
                checksum = crc_calculator.calculate_checksum(identifier + msg_packed)
                checksum_packed = self._checksum_struct.pack(checksum)
                # print(identifier + msg_packed + checksum_packed)
                ser.write(identifier + msg_packed + checksum_packed)
        ser.close()

    def set_callback(self, cb_func):
        """
        Set callback function for when data from motors arrives.
        The data in the form of a dictionary will be pass to this function.

        :param cb_func: function with one argument
        """
        self._on_motor_measurement_cb = cb_func
        self._write_to_queue = False

    def remove_callback(self):
        """
        Remove callback function
        """
        self._on_motor_measurement_cb = None
        self._write_to_queue = True

    def disconnect(self):
        """
        Disconnect serial.
        """
        self._stop_serial = True
        self._thread.join()


if __name__ == "__main__":
    c = Controller()
    c.read_data = True
    c.set_speed(1, 50)
