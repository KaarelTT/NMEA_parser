import pyvisa

def list_visa_devices():
    rm = pyvisa.ResourceManager()
    devices = rm.list_resources()
    return devices

if __name__ == "__main__":
    devices = list_visa_devices()
    for device in devices:
        print(device)
