import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
import time
import os

from custom_msg.msg import BMS # Import the BMS custom message

# Configure Modbus Serial Client
usb_port = '/dev/ttyBMS' # TOP RIGHT OF PI !
bms_available = True

client = ModbusSerialClient(
    port=usb_port, timeout=2, baudrate=115200
)

if not client.connect():
     print("Failed to connect to Modbus device.")
     bms_available = False
#     # exit(1)


class BMSPublisher(Node):
    def __init__(self):
        super().__init__('BMS_publisher') # Super init calls the node class's constructor
        self.publisher_ = self.create_publisher(BMS, '/EL/bms_topic', 10) # Create a publisher on the topic BMS_topic
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if(bms_available):
            v_bat,status,current = update()
            msg = BMS()
            msg.v_bat = float(v_bat)
            msg.status = status
            msg.current = float(current)
        else:
            msg = BMS()
            msg.v_bat = float(0)
            msg.status = "disconnected"
            msg.current = float(0)
            
        self.publisher_.publish(msg) # Publish the message on the topic

def main(args=None):
    rclpy.init(args=args)

    bms_pub = BMSPublisher() # Create an instance of the BMSPublisher class

    rclpy.spin(bms_pub) # spin the bms publisher node

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bms_pub.destroy_node()
    rclpy.shutdown() # When finished

# Helper function to read registers
def read_registers(client, address, count, slave_id):
    response = client.read_holding_registers(address=address, count=count, slave=slave_id)
    if response.isError():
        raise Exception(f"Error reading registers at address {address}: {response}")
    return response.registers

# Periodic update function
def update():
    try:
        # Get time stamp
        sample_time = datetime.datetime.now()

        # Read balance state and encode as binary
        registers = read_registers(client, 51, 16, slave_id=0xAA)
        balance = registers[0] if registers else 0
        balances = '{:016b}'.format(balance)

        # Read pack current and temperature
        registers = read_registers(client, 38, 2, slave_id=0xAA)
        current = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0] if registers else 0)

        registers = read_registers(client, 36, 2, slave_id=0xAA)
        v_bat = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0] if registers else 0)

        #read status
        registers = read_registers(client, 50, 1, slave_id=0xAA)
        match registers[0]:
            case 0x91:
                status= "Charging"
            case 0x92:
                status="Fully-Charged"
            case 0x93:
                status = "Discharging"
            case 0x96:
                status ="Regeneration"
            case 0x97:
                status ="Idle"
            case 0x9B:
                status ="Fault"
            case _ :
                status = "comm err"

    except Exception as e:
        print(f"Error during update: {e}")
        return (0," unknown",0)
    
    return (v_bat,status,current)
