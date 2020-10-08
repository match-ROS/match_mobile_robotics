import struct
import socket
import threading
from net_box_hardware_helper import NetBoxEnums as enum

class NetBoxSocket:
    def __init__(self, ip_adress, port):
        self.__adress = (ip_adress, port)
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__msg_gen = NetBoxMessageGenerator()
        self.__sample_gen = NetBoxSampleGenerator(1000000,1000000)
        self.__scope=threading.Thread(target=self.__scope__)
        self.__streaming=False
        self.__data=NetBoxMeasurement()
        self.__receiveCb=None

    def startStreaming(self):
        data = self.__msg_gen(enum.START_RT_STREAMING)
        self.__socket.sendto(data,self.__adress)
        self.__streaming = True
        self.__scope.start()
       
    def registerReceiveCb(self,callback):
        self.__receiveCb=callback

    def stopStreaming(self):
        data = self.__msg_gen(enum.STOP_STREAMING)
        self.__streaming = False
        self.__socket.sendto(data,self.__adress)
        
        



    def __scope__(self):
        while self.__streaming:
            data=self.__socket.recv(4096)
            self.__data=self.__sample_gen(data)
            if self.__receiveCb:
                self.__receiveCb(self.__data)
        



class NetBoxMessageGenerator:
    def __init__(self, extended=False):
        if not extended:
            self.__packer=struct.Struct('> H H I')
        self.__data = bytes()

    def __repr__(self):
        return ''.join(format(ord(byte), '08b') for byte in self.__data)

    def __call__(self, *argv):
        if len(argv) == 1:
            self.__data=self.__packer.pack(enum.START_IDENTIFIER, argv[0], 0)
            return self.__data
        elif len(argv) == 2:
            self.__data = self.__packer.pack(enum.START_IDENTIFIER, argv[0], argv[1])
            return self.__data

class NetBoxSampleGenerator:
    def __init__(self,f_counts,m_counts):
        self.__f_counts=f_counts
        self.__m_counts=m_counts
        self.__unpacker=struct.Struct('> I I I i i i i i i')


    def __call__(self,data):
        tupel_data=self.__unpacker.unpack(data)
        measurement_data=NetBoxMeasurement()
        measurement_data.Fx = float(tupel_data[3]) / self.__f_counts
        measurement_data.Fy = float(tupel_data[4]) / self.__f_counts
        measurement_data.Fz = float(tupel_data[5]) / self.__f_counts
        measurement_data.Mx = float(tupel_data[6]) / self.__m_counts
        measurement_data.My = float(tupel_data[7]) / self.__m_counts
        measurement_data.Mz = float(tupel_data[8]) / self.__m_counts
        return measurement_data
   


class NetBoxMeasurement:
    seq=long()
    Fx=float()
    Fy=float()
    Fz=float()
    Mx=float()
    My=float()
    Mz=float()
    
    def __repr__(self):
        string= "Seq:\t"+str(self.seq)+\
                "\tFx:\t"+str(self.Fx)+\
                "\tFy:\t"+str(self.Fy)+\
                "\tFz:\t"+str(self.Fz)+\
                "\tMx:\t"+str(self.Mx)+\
                "\tMy:\t"+str(self.My)+\
                "\tMz:\t"+str(self.Mz)
                
        return string 



