import logging
import re
import struct
import threading
from collections import deque
from functools import wraps
from queue import Queue
from time import sleep
from typing import Callable

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
                logging.error("No valid port found.")
                raise ValueError("No valid port found.")
        else:
            self.port = port

        self.baud = baud

        self._read_deque = deque(maxlen=sampling_rate)

        self._thread = None
        self._write_to_queue = True
        self._on_motor_measurement_cb = None
        self._stop_serial = threading.Event()
        self._serial_connected = threading.Event()

        self._header = data_variables
        self._read_struct = struct.Struct(_read_format)
        self._write_struct = struct.Struct(_write_format)
        self._checksum_struct = struct.Struct(">H")

        self._selected_control_mode = ControlMode.Speed_Control

        self._write_queue = Queue()
        self._start_serial()

    @staticmethod
    def if_motor_no_is_valid(func):
        @wraps(func)
        def wrapper(self, motor_no, *args, **kwargs):
            if motor_no not in [1, 2]:
                logging.error("Invalid motor no.")
                raise ValueError("Invalid motor no.")
            return func(self, motor_no, *args, **kwargs)

        return wrapper

    @staticmethod
    def if_no_cb_assigned(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not self._write_to_queue:
                logging.error("Callback has been assigned.")
                raise RuntimeError("Callback has been assigned.")
            return func(self, *args, **kwargs)

        return wrapper

    @staticmethod
    def wait_for_data(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            while not self._read_deque:
                sleep(0.01)
            return func(self, *args, **kwargs)

        return wrapper

    @staticmethod
    def log_command(func):
        @wraps(func)
        def wrapper(self, motor_no, *args, **kwargs):
            val = "".join(str(x) for x in (*args, *kwargs.values()))
            command = func.__doc__.lstrip().split("\n", 1)[0][:-1]
            if not val:
                logging.info(f"{command} for motor {motor_no}.")
            else:
                logging.info(f"{command} for motor {motor_no} to {val}.")
            return func(self, motor_no, *args, **kwargs)

        return wrapper

    @log_command
    @if_motor_no_is_valid
    def set_control_mode(self, motor_no: int, mode: str):
        """
        Set motor control mode.

        :param motor_no: 1 or 2
        :param mode: Choose between 'speed control', 'torque control', 'position control (speed)', 'position control (direct)', 'bang bang control', and 'impedance control' modes
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
                case "impedance control with load" | "impedance with load":
                    self._selected_control_mode = (
                        ControlMode.Impedance_Control_With_Load
                    )
                case _:
                    logging.error("Invalid control mode.")
                    raise ValueError("Invalid control mode.")

        elif isinstance(mode, (ControlMode, int)):
            self._selected_control_mode = mode

        else:
            raise ValueError("invalid control mode")

        self._write_queue.put((b"\x01", motor_no, float(self._selected_control_mode)))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_max_torque(self, motor_no: int, torque: float):
        """
        Set maximum torque in N m.
        Used in position control (speed), position control (direct), bang bang, and impedance control modes.

        :param motor_no: 1 or 2
        :param torque: Torque in N m (max torque = 4.3 N m)
        """
        self._write_queue.put((b"\x02", motor_no, torque))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_speed(self, motor_no: int, speed: float):
        """
        Set target speed.
        Used in speed control mode.

        :param motor_no: 1 or 2
        :param speed: Rotational speed in rad/s (max speed = 30 rad/s)
        """
        self._write_queue.put((b"\x03", motor_no, speed))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos(self, motor_no: int, pos: float):
        """
        Set target pos.
        Used in position control (speed), position control (direct), bang bang, and impedance control modes.

        :param motor_no: 1 or 2
        :param pos: Target angle in radians
        """
        self._write_queue.put((b"\x04", motor_no, pos))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_current_measure_cutoff_freq(self, motor_no: int, freq: float):
        """
        Set the cutoff frequency for iq current filter.
        Used when estimating motor torque from phase currents.

        :param motor_no: 1 or 2
        :param freq: Cutoff frequency in Hz
        """
        self._write_queue.put((b"\x05", motor_no, freq))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_torque_sensor_cutoff_freq(self, motor_no: int, freq: float):
        """
        Set the cutoff frequency for torque sensor filter.
        Used when measuring torque from torque sensor.

        :param motor_no: 1 or 2
        :param freq: Cutoff frequency in Hz
        """
        self._write_queue.put((b"\x06", motor_no, freq))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_torque_sensor_scaling(self, motor_no: int, gradient: float):
        """
        Set the gradient of torque sensor calibration curve.
        Used for calibrating torque sensor.

        :param motor_no: 1 or 2
        :param gradient: gradient of torque sensor calibration line
        """
        self._write_queue.put((b"\x07", motor_no, gradient))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_torque_sensor_offset(self, motor_no: int, offset: float):
        """
        Set the offset of torque sensor calibration curve.
        Used for calibrating torque sensor.

        :param motor_no: 1 or 2
        :param offset: offset of torque sensor calibration line
        """
        self._write_queue.put((b"\x08", motor_no, offset))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_direct_torque_p(self, motor_no: int, gain: float):
        """
        Set P gain for direct torque control.
        Used to regulate motor torque to achieve sensor torque target.

        :param motor_no: 1 or 2
        :param gain: Proportional gain
        """
        self._write_queue.put((b"\x09", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_direct_torque_i(self, motor_no: int, gain: float):
        """
        Set I gain for direct torque control.
        Used to regulate motor torque to achieve sensor torque target.

        :param motor_no: 1 or 2
        :param gain: Integral gain
        """
        self._write_queue.put((b"\x0A", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_direct_torque_d(self, motor_no: int, gain: float):
        """
        Set D gain for direct torque control.
        Used to regulate motor torque to achieve sensor torque target.

        :param motor_no: 1 or 2
        :param gain: Derivative gain
        """
        self._write_queue.put((b"\x0B", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_direct_torque_d_cutoff_freq(self, motor_no: int, freq: float):
        """
        Set the derivative filter cutoff frequency (Hz) for direct torque control.
        Used to regulate motor torque to achieve sensor torque target.

        :param motor_no: 1 or 2
        :param freq: Cutoff frequency in Hz
        """
        self._write_queue.put((b"\x0C", motor_no, freq))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_speed_p(self, motor_no: int, gain: float):
        """
        Set P gain for speed control.
        Used in speed control and position control (speed) modes.

        :param motor_no: 1 or 2
        :param gain: Proportional gain
        """
        self._write_queue.put((b"\x11", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_speed_i(self, motor_no: int, gain: float):
        """
        Set I gain for speed control.
        Used in speed control and position control (speed) modes.

        :param motor_no: 1 or 2
        :param gain: Integral gain
        """
        self._write_queue.put((b"\x12", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_speed_d(self, motor_no: int, gain: float):
        """
        Set D gain for speed control.
        Used in speed control and position control (speed) modes.

        :param motor_no: 1 or 2
        :param gain: Derivative gain
        """
        self._write_queue.put((b"\x13", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_speed_d_cutoff_freq(self, motor_no: int, freq: float):
        """
        Set the derivative filter cutoff frequency (Hz) for speed control.
        Used in speed control and position control (speed) modes.

        :param motor_no: 1 or 2
        :param freq: Cutoff frequency in Hz
        """
        self._write_queue.put((b"\x14", motor_no, freq))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_hold_torque(self, motor_no: int, torque: float):
        """
        Set constant torque.
        Used in torque control mode and impedance control with torque mode.

        :param motor_no: 1 or 2
        :param torque: Torque in N m (max torque = 4.3 N m)
        """
        self._write_queue.put((b"\x21", motor_no, torque))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_p(self, motor_no: int, gain: float):
        """
        Set P gain for position control (speed).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param gain: Proportional gain
        """
        self._write_queue.put((b"\x31", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_i(self, motor_no: int, gain: float):
        """
        Set I gain for position control (speed).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param gain: Integral gain
        """
        self._write_queue.put((b"\x32", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_d(self, motor_no: int, gain: float):
        """
        Set D gain for position control (speed).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param gain: Derivative gain
        """
        self._write_queue.put((b"\x33", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_d_cutoff_freq(self, motor_no: int, freq: float):
        """
        Set the derivative filter cutoff frequency (Hz) for position control (speed).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param freq: Cutoff frequency in Hz
        """
        self._write_queue.put((b"\x34", motor_no, freq))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_max_speed(self, motor_no: int, speed: float):
        """
        Set maximum target speed for position control.
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param speed: Rotational speed in rad/s (max speed = 30 rad/s)
        """
        self._write_queue.put((b"\x35", motor_no, speed))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_p_direct(self, motor_no: int, gain: float):
        """
        Set P gain for position control (direct).
        Used in position control (direct) mode.

        :param motor_no: 1 or 2
        :param gain: Proportional gain
        """
        self._write_queue.put((b"\x41", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_i_direct(self, motor_no: int, gain: float):
        """
        Set I gain for position control (direct).
        Used in position control (direct) mode.

        :param motor_no: 1 or 2
        :param gain: Integral gain
        """
        self._write_queue.put((b"\x42", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_d_direct(self, motor_no: int, gain: float):
        """
        Set D gain for position control (direct).
        Used in position control (direct) mode.

        :param motor_no: 1 or 2
        :param gain: Derivative gain
        """
        self._write_queue.put((b"\x43", motor_no, gain))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_pos_d_direct_cutoff_freq(self, motor_no: int, freq: float):
        """
        Set the derivative filter cutoff frequency (Hz) for position control (direct).
        Used in position control (speed) mode.

        :param motor_no: 1 or 2
        :param freq: Cutoff frequency in Hz
        """
        self._write_queue.put((b"\x44", motor_no, freq))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_bang_bang_torque(self, motor_no: int, torque: float):
        """
        Set constant torque in bang-bang control.
        Used in bang-bang control mode.

        :param motor_no: 1 or 2
        :param torque: Torque in N m (max torque = 4.3 N m)
        """
        self._write_queue.put((b"\x51", motor_no, torque))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_bang_bang_deadband(self, motor_no: int, deadband: float):
        """
        Set deadband in radians in bang-bang control.
        Used in bang-bang control mode.

        :param motor_no: 1 or 2
        :param deadband: Deadband in radians
        """
        self._write_queue.put((b"\x52", motor_no, deadband))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_inertia(self, motor_no: int, inertia: float):
        """
        Set rotational inertia.
        Used in impedance control mode.

        :param motor_no: 1 or 2
        :param inertia: Inertia
        """
        self._write_queue.put((b"\x61", motor_no, inertia))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_damping(self, motor_no: int, damping: float):
        """
        Set rotational damping.
        Used in impedance control mode.

        :param motor_no: 1 or 2
        :param damping: Damping
        """
        self._write_queue.put((b"\x62", motor_no, damping))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_stiffness(self, motor_no: int, stiffness: float):
        """
        Set rotational stiffness.
        Used in impedance control mode.

        :param motor_no: 1 or 2
        :param stiffness: Stiffness
        """
        self._write_queue.put((b"\x63", motor_no, stiffness))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_zero_angle(self, motor_no: int):
        """
        Set zero angle.
        Used in position control (speed), position control (direct), bang bang, and impedance control modes.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF0", motor_no, 1))
        return self

    @log_command
    @if_motor_no_is_valid
    def realign_motor(self, motor_no: int):
        """
        Realign motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF2", motor_no, 1))
        return self

    @log_command
    @if_motor_no_is_valid
    def enable_motor(self, motor_no: int):
        """
        Enable motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF3", motor_no, 1))
        return self

    @log_command
    @if_motor_no_is_valid
    def disable_motor(self, motor_no: int):
        """
        Disable motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF3", motor_no, 0))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_match_other_position(self, motor_no: int):
        """
        Set motor target position to current position of the other motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF4", motor_no, 1))
        return self

    @log_command
    @if_motor_no_is_valid
    def unset_match_other_position(self, motor_no: int):
        """
        Unset motor target position to current position of the other motor.

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF4", motor_no, 0))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_friction_torque(self, motor_no: int, torque: float):
        """
        Torque value for friction compensation.

        :param motor_no: 1 or 2
        :param torque: Friction compensation torque in N m
        """
        self._write_queue.put((b"\xF5", motor_no, torque))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_friction_vel_tolerance(self, motor_no: int, vel_tolerance: float):
        """
        Velocity tolerance for friction compensation.

        :param motor_no: 1 or 2
        :param vel_tolerance: Velocity in rad s-1
        """
        self._write_queue.put((b"\xF6", motor_no, vel_tolerance))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_direct_torque_on(self, motor_no: int):
        """
        Set direct torque control on.
        (Adjust motor torque to target a torque sensor value)

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF7", motor_no, 1))
        return self

    @log_command
    @if_motor_no_is_valid
    def set_direct_torque_off(self, motor_no: int):
        """
        Set direct torque control off.
        (Adjust motor torque to target a torque sensor value)

        :param motor_no: 1 or 2
        """
        self._write_queue.put((b"\xF7", motor_no, 0))
        return self

    def _start_serial(self):
        self._thread = threading.Thread(target=self._read_serial_data, daemon=True)
        self._thread.start()
        while not self._serial_connected.is_set():
            sleep(0.01)

    def _read_serial_data(self):
        ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=1)
        logging.info(f"Connected to {self.port} at baudrate {self.baud}.")
        self._serial_connected.set()

        while not self._stop_serial.is_set():
            raw_data = ser.read(self._read_struct.size)
            if not raw_data:
                continue
            unpacked_data = self._read_struct.unpack(raw_data)
            if unpacked_data[-1] != b"\n":
                _ = ser.readline()
            elif self._write_to_queue:
                self._read_deque.append(dict(zip(self._header, unpacked_data[1:-1])))
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

    def set_callback(self, cb_func: Callable[[dict], None]):
        """
        Set callback function for when data from motors arrives.
        The data in the form of a dictionary will be pass to this function.

        :param cb_func: function with one argument
        """
        self._on_motor_measurement_cb = cb_func
        self._write_to_queue = False
        logging.info(f"Callback set to {cb_func.__name__}.")
        return self

    def remove_callback(self):
        """
        Remove callback function.
        """
        self._on_motor_measurement_cb = None
        self._read_deque.clear()
        self._write_to_queue = True
        logging.info("Callback removed.")
        return self

    @if_no_cb_assigned
    @wait_for_data
    def get_from_queue(self):
        """
        Data from motors are put into a read queue by default.
        To get data from motors, either read from this queue or set a callback function.
        """
        return self._read_deque.popleft()

    @log_command
    @if_no_cb_assigned
    @if_motor_no_is_valid
    @wait_for_data
    def get_pos(self, motor_no: int) -> float:
        """
        Get current pos.

        :param motor_no: 1 or 2
        :return: Current position in radians
        """
        key = f"motor{motor_no}_pos"
        return self._read_deque[-1][key]

    @log_command
    @if_no_cb_assigned
    @if_motor_no_is_valid
    @wait_for_data
    def get_speed(self, motor_no: int) -> float:
        """
        Get current speed.

        :param motor_no: 1 or 2
        :return: Current speed in rad/s
        """
        key = f"motor{motor_no}_speed"
        return self._read_deque[-1][key]

    @log_command
    @if_no_cb_assigned
    @if_motor_no_is_valid
    @wait_for_data
    def get_torque(self, motor_no: int) -> float:
        """
        Get current torque.

        :param motor_no: 1 or 2
        :return: Current torque in N m
        """
        key = f"motor{motor_no}_torque"
        return self._read_deque[-1][key]

    def disconnect(self):
        """
        Disconnect serial.
        """
        self._stop_serial.set()
        self._thread.join()
        self._serial_connected.clear()
        self._stop_serial.clear()

        logging.info("Disconnected.")
        return self


if __name__ == "__main__":
    c = Controller()
    c.set_speed(1, 50)
