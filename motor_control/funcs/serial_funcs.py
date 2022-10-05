from serial.tools import list_ports


def find_port(device_name):
    for port, desc, _ in list_ports.comports():
        if device_name in desc:
            break
    else:
        port = None
    return port
