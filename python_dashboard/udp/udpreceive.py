
import socket
import sys
import struct

def map_range(x, in_min=0, in_max=65536, out_min=-180, out_max=180):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def convert_to_8bit_unsigned(data_packet):
    """Converts a data packet into a list of 8-bit unsigned integers."""

    # Unpack the data packet into a list of unsigned bytes
    unpacked_data = struct.unpack('B' * len(data_packet), data_packet)

    return unpacked_data

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except socket.error as msg:
    print("Failed to create socket. Error Code : " + str(msg[0]) + " Message " + msg[1])
    sys.exit()

try:
    s.bind(("", 4444))
except socket.error as msg:

    print("Bind failed. Error: " + str(msg[0]) + ": " + msg[1])
    sys.exit()

print("Server listening")

print("Server listening")

while 1:
    d = s.recvfrom(1024)
    data = d[0]

    if not data:
        break

    datalist = convert_to_8bit_unsigned(data)
    # print(datalist)
    print("GyX = ",map_range(data[3]<<8|data[4]),"GyY = ",map_range(data[5]<<8|data[6]),"GyZ = ",map_range(data[7]<<8|data[8]))

s.close()
