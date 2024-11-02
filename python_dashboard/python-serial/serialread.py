import serial

def find_available_ports():
    ports = []
    for i in range(256):
        port = f"/dev/ttyUSB{i}"
        try:
            s = serial.Serial(port)
            s.close()
            ports.append(port)
        except (OSError, serial.SerialException):
            pass
    return ports

def read_serial_data(port):
    ser = serial.Serial(port, baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1)

    while True:
        data = ser.read(14)
        datalist = list(data) #process the list further for getting individual slaves and training the data

if __name__ == "__main__":
    available_ports = find_available_ports()
    if available_ports:
        port = available_ports[0] #chooses the first available port in case of more than one serial devices connected ie(/dev/ttyUSB0 and /dev/ttyUSB1) it chooses the first one ie(/dev/ttyUSB0) edit the code according to your use case
        read_serial_data(port)
    else:
        print("No serial ports found.")