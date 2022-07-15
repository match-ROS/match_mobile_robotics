#!/usr/bin/env python3
from net_box_hardware_helper import NetBoxEnums as enum
from net_box_hardware_helper import NetBoxMessageGenerator
from net_box_hardware_helper import NetBoxSocket
from net_box_hardware_helper import NetBoxSampleGenerator
import time


def recCb(data):
    print(data)

print("Enums:")
print(enum.STOP_STREAMING)
print(enum.START_RT_STREAMING)
print(enum.START_BUFFERED_STREAMING)
print(enum.RESET_TRESH_LATCH)
print(enum.SET_SOFTWARE_BIAS)
print(enum.START_IDENTIFIER)

socket=NetBoxSocket('192.168.13.1', 49152)
socket.registerReceiveCb(recCb)

print("initial stop")
socket.stopStreaming()
time.sleep(3)
print("initial start")
socket.startStreaming()
time.sleep(3)
socket.stopStreaming()
print("Set bias")
socket.setSoftwareBias()
time.sleep(3)
socket.startStreaming()
time.sleep(3)
socket.stopStreaming()
print("stopped")



