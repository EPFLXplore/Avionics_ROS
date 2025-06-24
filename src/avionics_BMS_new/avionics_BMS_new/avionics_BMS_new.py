import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
import time
import os
# Import the BMS custom message
from custom_msg.msg import BMS 

usb_port = '/dev/ttyBMS' # TOP RIGHT OF PI !

class BMSPublisher(Node):
    def __init__(self):
        super().__init__('BMS_publisher')
        self.publisher_ = self.create_publisher(BMS, '/EL/bms_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.usb_port = usb_port
        self.client = None
        self.bms_available = False
        self.reconnect_counter = 0
        self.reconnect_interval = 2 
        self.try_connect()
    
    def try_connect(self):
        try:
            self.client = ModbusSerialClient(port=self.usb_port, timeout=2, baudrate=115200)
            if self.client.connect():
                self.get_logger().info("Connected to Modbus device.")
                self.bms_available = True
            else:
                self.get_logger().warn("Failed to connect to Modbus device.")
                self.bms_available = False
        except Exception as e:
            self.get_logger().error(f"Exception during connect: {e}")
            self.client = None
            self.bms_available = False

    def try_reconnect(self):
        self.get_logger().info("Attempting to reconnect to Modbus device...")
        self.try_connect()

    def read_registers(self, address, count, slave_id):
        if not self.client:
            raise Exception("Client is not initialized.")
        response = self.client.read_holding_registers(address=address, count=count, slave=slave_id)
        if response.isError():
            raise Exception(f"Error reading registers at address {address}: {response}")
        return response.registers

    def update(self):
        try:
            # registers = self.read_registers(51, 16, slave_id=0xAA)
            # balance = '{:016b}'.format(registers[0]) if registers else '0'*16
 
            registers = self.read_registers(38, 2, slave_id=0xAA)
            current = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0]) if registers else 0.0

            registers = self.read_registers(36, 2, slave_id=0xAA)
            v_bat = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0]) if registers else 0.0

            registers = self.read_registers(50, 1, slave_id=0xAA)
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
            return (v_bat,status,current)

        except Exception as e:
            self.get_logger().warn(f"Error during update: {e}")
            self.bms_available = False
            return (0.0, "disconnected", 0.0)

    def timer_callback(self):
        if not self.bms_available:
            self.reconnect_counter += 1
            if self.reconnect_counter >= self.reconnect_interval:
                self.reconnect_counter = 0
                self.try_reconnect()

        if(self.bms_available):
            v_bat,status,current = self.update()
        else:
            v_bat, status, current = 0.0, "disconnected", 0.0

        msg = BMS()
        msg.v_bat = float(v_bat)
        msg.status = status
        msg.current = float(current)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    bms_pub = BMSPublisher() 
    rclpy.spin(bms_pub)
    bms_pub.destroy_node()
    rclpy.shutdown()

