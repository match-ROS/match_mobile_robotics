#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String
import subprocess
import asyncio
from typing import List
import can
from can.notifier import MessageRecipient
import std_msgs
from math import floor

import time


#LED_PIN = board.D21 # LED control pin 
#BATTERY_NODE_ID = 0x0240  # address assignment of the message ->  0x<NODE_ID><40>   
                          # <NODE_ID> is written on the battery


can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 250000
P = 0x18 
bms_SOC = 0

def init():
    global BATTERY_NODE_ID
    BATTERY_NODE_ID = rospy.get_param('~battery_node_id', "0x0240")  # address assignment of the message ->  0x<NODE_ID><40>
    # convert the string to an integer
    
    if isinstance(BATTERY_NODE_ID, str):
        if BATTERY_NODE_ID.startswith("0x"):
            BATTERY_NODE_ID = int(BATTERY_NODE_ID, 16)
        else:
            BATTERY_NODE_ID = int(BATTERY_NODE_ID, 10)


def message_callback(msg: can.Message) -> None:
    """Regular callback function. Can also be a coroutine."""
    data_ID = hex(msg.arbitration_id)[4:6] 
    global bms_SOC
    if data_ID == "90":
        Cumulative_total_voltage = int(msg.data[0:2].hex(),16)/10
        Gather_total_voltage = int(msg.data[2:4].hex(),16)/10
        Current = int(msg.data[4:6].hex(),16)/10 - 3000
        bms_SOC = int(msg.data[6:8].hex(),16)/10

# function gets the data ID and calls process it
async def update_can(d_id):
    with can.Bus() as bus:
        reader = can.AsyncBufferedReader()
        logger = can.Logger("logfile.asc")

        listeners: List[MessageRecipient] = [
            message_callback,  # Callback function
            reader,  # AsyncBufferedReader() listener
            logger,  # Regular Listener object
        ]
        # Create Notifier with an explicit loop to use for scheduling of callbacks
        loop = asyncio.get_running_loop()
        notifier = can.Notifier(bus, listeners, loop=loop)
        # Start sending first message
        a1 = (P << 8) | (d_id) 
        can_id = (a1 << 16 ) | BATTERY_NODE_ID
        msg_content = []
        try:
            bus.send(can.Message(arbitration_id=can_id, data=msg_content, is_extended_id=True)) 
        except can.CanError as e :
            rospy.logerr(e)
        try:
            await asyncio.wait_for(reader.get_message(), 10)
        except asyncio.TimeoutError as e: 
            rospy.logerr(e)
        notifier.stop()

def update_bms():
    asyncio.run(update_can(0x90)) # message ID that is sent to the BMS


def setup_can_interface():
    """Setzt die CAN-Schnittstelle auf 'can0' mit der Bitrate von 250000."""
    try:
        command = f"echo match123 | sudo -S ip link set can0 up type can bitrate 250000" # TODO: do not hardcode password - find a more secure method
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Fehler beim Aktivieren des CAN Interfaces: {e}")


if __name__ == '__main__':
    setup_can_interface()
    rospy.init_node('bms_manager_node', log_level=rospy.DEBUG)
    rospy.loginfo("bms_node_node started")
    init()
    try:
        pub = rospy.Publisher('bms_status/SOC', std_msgs.msg.Float32, queue_size=10)
        rate = rospy.Rate(1) # refresh every second
        while not rospy.is_shutdown():
            try: 
                update_bms()
            except TimeoutError as e:
                rospy.logerr(e)
            except can.CanError as e:
                rospy.logerr(e)
            SOC_msg = "SOC is {}%".format(bms_SOC)
            #rospy.logdebug(SOC_msg)
            pub.publish(round(bms_SOC,1))
            rate.sleep()
    finally:    
        rospy.loginfo("bms_manager_node shut down")
        
        