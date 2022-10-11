from f28379d_drv8305_dual_comm import Controller

c = Controller()  # connect to controller

for _ in range(500):
    print(c.get_from_queue())

c.disconnect()  # disconnect from controller
