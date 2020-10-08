#!/usr/bin/env python
import struct
import binascii
import socket
import sys

COUNTS_FORCE=1000000
COUNTS_TORQUE=1000000

def printBits(bytes_in):
    print(''.join(format(ord(byte), '08b') for byte in bytes_in))

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('192.168.1.1', 49152)

start= 0x1234
command=0x0002
sample_num=0

msg=struct.pack( ">H H I ", start,command ,sample_num )


printBits(msg)
sock.sendto(msg,server_address)

while True:
    data, address = sock.recvfrom(4096)
    data_formated=struct.unpack('> I I I i i i i i i',data)
    data_formated=list(data_formated)
    data_formated[3:6]=(float(x) /COUNTS_FORCE for x in data_formated[3:6])
    data_formated[6:9]=(float(x) /COUNTS_TORQUE for x in data_formated[6:9])
    
    print(data_formated)