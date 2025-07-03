import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
import minimalmodbus
import serial
import time
import os

from rclpy.executors import MultiThreadedExecutor
from custom_msg.msg import BMS, FourInOne, LEDMessage

usb_port_bms = '/dev/ttyBMS' # TOP RIGHT OF PI !
usb_port_4in1 = '/dev/tty4in1'
usb_port_leds = '/dev/ttyESP32_LED'

# 4in1 register definitions
HUM_REGISTER = 0
TEMP_REGISTER = 1
EC_REGISTER = 2
PH_REGISTER = 3

class PythonPublisher(Node):
    def __init__(self):
        super().__init__('python_publisher')
        self.publisher_bms = self.create_publisher(BMS, '/EL/bms_topic', 10)
        self.publisher_4in1 = self.create_publisher(FourInOne, '/EL/FourInOne_topic', 10)

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # BMS Setup
        self.usb_port_bms = usb_port_bms
        self.client_bms = None
        self.bms_available = False
        self.bms_reconnect_counter = 0
        self.bms_reconnect_interval = 2 
        self.try_connect_bms()
        self.read_BMS()
        self.try_reconnect_bms()

        # 4in1 Setup
        self.usb_port_4in1 = usb_port_4in1
        self.instrument_4in1 = None
        self.FourinOne_available = False
        self.FourinOne_reconnect_counter = 0
        self.FourinOne_reconnect_interval = 2 
        self.try_connect_4in1()
        self.read_4in1()
        self.try_reconnect_4in1()
    
    def try_connect_bms(self):
        try:
            self.client_bms = ModbusSerialClient(port=self.usb_port_bms, timeout=2, baudrate=115200)
            if self.client_bms.connect():
                self.get_logger().info("Connected to BMS device.")
                self.bms_available = True
            else:
                self.get_logger().warn("Failed to connect to BMS device.")
                self.bms_available = False
        except Exception as e:
            self.get_logger().error(f"Exception during BMS connect: {e}")
            self.client_bms = None
            self.bms_available = False

    def try_reconnect_bms(self):
        self.get_logger().info("Attempting to reconnect to Modbus device...")
        self.try_connect_bms()

    def read_registers(self, address, count, slave_id):
        if not self.client_bms:
            raise Exception("BMS Client is not initialized.")
        response = self.client_bms.read_holding_registers(address=address, count=count, slave=slave_id)
        if response.isError():
            raise Exception(f"BMS Error reading registers at address {address}: {response}")
        return response.registers

    def read_BMS(self):
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
            self.get_logger().warn(f"BMS Error during update: {e}")
            self.bms_available = False
            return (0.0, "disconnected", 0.0)

    def try_connect_4in1(self):
        try:
            self.instrument_4in1 = minimalmodbus.Instrument(self.usb_port_4in1, 1, mode=minimalmodbus.MODE_RTU)
            self.instrument_4in1.serial.baudrate = 9600
            self.instrument_4in1.serial.bytesize = 8
            self.instrument_4in1.serial.parity = serial.PARITY_NONE
            self.instrument_4in1.serial.stopbits = 1
            self.instrument_4in1.serial.timeout = 1 # seconds
            self.instrument_4in1.close_port_after_each_call = True
            self.instrument_4in1.clear_buffers_before_each_transaction = True

            # Try a read to verify
            self.instrument_4in1.read_register(TEMP_REGISTER, number_of_decimals=1)
            self.FourinOne_available = True
            self.get_logger().info("Connected to 4in1 sensor.")
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to 4in1 sensor: {e}")
            self.instrument_4in1 = None
            self.FourinOne_available = False

    def read_4in1(self):
        try:
            temperature = self.instrument_4in1.read_register(TEMP_REGISTER, number_of_decimals=1)
            humidity = self.instrument_4in1.read_register(HUM_REGISTER, number_of_decimals=1)
            ec = self.instrument_4in1.read_register(EC_REGISTER)
            ph = self.instrument_4in1.read_register(PH_REGISTER, number_of_decimals=1)
            return (temperature, humidity, ec, ph)
        except Exception as e:
            self.get_logger().warn(f"4in1 sensor read error: {e}")
            self.FourinOne_available = False
            return None

    def try_reconnect_4in1(self):
        self.get_logger().info("Attempting to reconnect to Modbus device...")
        self.try_connect_4in1()

    def timer_callback(self):
        if not self.bms_available:
            self.bms_reconnect_counter += 1
            if self.bms_reconnect_counter >= self.bms_reconnect_interval:
                self.bms_reconnect_counter = 0
                self.try_reconnect_bms()

        if not self.FourinOne_available:
            self.FourinOne_reconnect_counter += 1
            if self.FourinOne_reconnect_counter >= self.FourinOne_reconnect_interval:
                self.FourinOne_reconnect_counter = 0
                self.try_reconnect_4in1()

        if(self.bms_available):
            v_bat,status,current = self.read_BMS()
        else:
            v_bat, status, current = 0.0, "disconnected", 0.0

        msg_bms = BMS()
        msg_bms.v_bat = float(v_bat)
        msg_bms.status = status
        msg_bms.current = float(current)
        self.publisher_bms.publish(msg_bms)

        if(self.FourinOne_available):
            id = 0
            temperature, humidity, ec, ph = self.read_4in1()
        else:
            id, temperature, humidity, ec, ph = 99, 0.0, 0.0, 0.0, 0.0

        msg_4in1 = FourInOne()
        msg_4in1.id = id
        msg_4in1.temperature = round(float(temperature),1)
        msg_4in1.moisture = round(float(humidity),1)
        msg_4in1.conductivity = round(float(ec),0) 
        msg_4in1.ph = round(float(ph),1)
        self.publisher_4in1.publish(msg_4in1)


class PythonSubscriber(Node):
    def __init__(self):
        super().__init__('python_subscriber')
        self.subscription = self.create_subscription(LEDMessage,'/EL/LedCommands',
                                                     self.leds_callback, 10)
        self.subscription  # prevent unused variable warning
        self.port = usb_port_leds
        self.serial = None
        self.open_serial_port()
        self.get_logger().info("LED Subscriber node initialized and subscribing to /EL/LedCommands")

   

    def open_serial_port(self):
        try:
            self.serial = serial.Serial(self.port, baudrate=115200, timeout=1)
            self.get_logger().info(f"Serial port {self.port} opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.serial = None

    def leds_callback(self, msg):
        self.get_logger().info('Received LED message:')
        self.get_logger().info('System "%s"' % msg.system)
        self.get_logger().info('State "%s"' % msg.state)

        if self.serial != None:
            try:
                # Prepare the LED message
                if msg.system not in [0, 1, 2, 3]:
                    self.get_logger().error("Invalid system value. Must be 0, 1, 2, or 3.")
                    return
                if msg.state not in [0, 1, 2]:
                    self.get_logger().error("Invalid state value. Must be 0, 1, or 2.")
                    return
                if msg.state == 0:
                    mode = 5  # Off
                    self.get_logger().info("Turning off LEDs.")
                elif msg.state == 1:
                    mode = 0  # On
                    self.get_logger().info("Turning on LEDs.")
                elif msg.state == 2:
                    mode = 3  # Blinking
                    self.get_logger().info("Blinking LEDs.")

                led_message = f"0 100 {msg.system} {mode}\n"
                self.serial.write(led_message.encode('ascii'))
                self.get_logger().info("LED message sent successfully.")
                
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")
        else:
            self.get_logger().error("Serial port is not open. Cannot send LED message.")
            self.open_serial_port()



def main(args=None):
    rclpy.init(args=args)

    python_pub = PythonPublisher() 
    rclpy.spin(python_pub)
    python_pub.destroy_node()
    rclpy.shutdown()


# # import datetime
# # import numpy as np
# # import rclpy
# # from rclpy.node import Node
# # from pymodbus.client import ModbusSerialClient
# # import time
# # import os
# # # Import the BMS custom message
# # from custom_msg.msg import BMS 

# # usb_port = '/dev/ttyBMS' # TOP RIGHT OF PI !

# class BMSPublisher(Node):
#     def __init__(self):
#         super().__init__('BMS_publisher')
#         self.publisher_ = self.create_publisher(BMS, '/EL/bms_topic', 10)
#         timer_period = 1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.usb_port = usb_port
#         self.client = None
#         self.bms_available = False
#         self.reconnect_counter = 0
#         self.reconnect_interval = 2 
#         self.try_connect()
    
#     def try_connect(self):
#         try:
#             self.client = ModbusSerialClient(port=self.usb_port, timeout=2, baudrate=115200)
#             if self.client.connect():
#                 self.get_logger().info("Connected to Modbus device.")
#                 self.bms_available = True
#             else:
#                 self.get_logger().warn("Failed to connect to Modbus device.")
#                 self.bms_available = False
#         except Exception as e:
#             self.get_logger().error(f"Exception during connect: {e}")
#             self.client = None
#             self.bms_available = False

#     def try_reconnect(self):
#         self.get_logger().info("Attempting to reconnect to Modbus device...")
#         self.try_connect()

#     def read_registers(self, address, count, slave_id):
#         if not self.client:
#             raise Exception("Client is not initialized.")
#         response = self.client.read_holding_registers(address=address, count=count, slave=slave_id)
#         if response.isError():
#             raise Exception(f"Error reading registers at address {address}: {response}")
#         return response.registers

#     def update(self):
#         try:
#             # registers = self.read_registers(51, 16, slave_id=0xAA)
#             # balance = '{:016b}'.format(registers[0]) if registers else '0'*16
 
#             registers = self.read_registers(38, 2, slave_id=0xAA)
#             current = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0]) if registers else 0.0

#             registers = self.read_registers(36, 2, slave_id=0xAA)
#             v_bat = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0]) if registers else 0.0

#             registers = self.read_registers(50, 1, slave_id=0xAA)
#             match registers[0]:
#                 case 0x91:
#                     status= "Charging"
#                 case 0x92:
#                     status="Fully-Charged"
#                 case 0x93:
#                     status = "Discharging"
#                 case 0x96:
#                     status ="Regeneration"
#                 case 0x97:
#                     status ="Idle"
#                 case 0x9B:
#                     status ="Fault"
#                 case _ :
#                     status = "comm err"
#             return (v_bat,status,current)

#         except Exception as e:
#             self.get_logger().warn(f"Error during update: {e}")
#             self.bms_available = False
#             return (0.0, "disconnected", 0.0)

#     def timer_callback(self):
#         if not self.bms_available:
#             self.reconnect_counter += 1
#             if self.reconnect_counter >= self.reconnect_interval:
#                 self.reconnect_counter = 0
#                 self.try_reconnect()

#         if(self.bms_available):
#             v_bat,status,current = self.update()
#         else:
#             v_bat, status, current = 0.0, "disconnected", 0.0

#         msg = BMS()
#         msg.v_bat = float(v_bat)
#         msg.status = status
#         msg.current = float(current)

#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)

#     bms_pub = BMSPublisher() 
#     rclpy.spin(bms_pub)
#     bms_pub.destroy_node()
#     rclpy.shutdown()
