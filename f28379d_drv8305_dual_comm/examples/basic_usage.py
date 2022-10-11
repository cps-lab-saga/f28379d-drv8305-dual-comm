from math import pi, degrees

from f28379d_drv8305_dual_comm import Controller

c = Controller()  # connect to controller

c.set_control_mode(1, "position control (speed)")  # set control mode for motor 1
c.disable_motor(2)  # disable motor 2

c.set_zero_angle(1)  # set current pos for motor 1 to zero

c.set_pos(1, pi)  # set target position of motor 1 to pi radians

# hold down the external pushbutton to run

current_pos = c.get_pos(1)  # get current position of motor 1
print(f"{degrees(current_pos)} degrees")

c.disconnect()  # disconnect from controller
