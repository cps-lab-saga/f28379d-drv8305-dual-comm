from time import sleep

from f28379d_drv8305_dual_comm import Controller


def my_callback_func(measurements):
    print(measurements)


c = Controller()  # connect to controller

c.set_callback(my_callback_func)  # function is called when data arrived from controller

sleep(5)

c.disconnect()  # disconnect from controller
