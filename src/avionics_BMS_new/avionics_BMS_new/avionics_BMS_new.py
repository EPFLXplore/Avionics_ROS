import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
import time
import os
# Import the BMS custom message
from custom_msg.msg import BMS 

usb_port_BMS = '/dev/ttyBMS' # TOP RIGHT OF PI !

usb_port_4in1 = '/dev/ttyUSB3'

HUM_REGISTER = 0
TEMP_REGISTER = 1
EC_REGISTER = 2
PH_REGISTER = 3

class BMSPublisher(Node):
    def __init__(self):
        super().__init__('BMS_publisher')
        self.publisher_ = self.create_publisher(BMS, '/EL/bms_topic', 10)
        self.publisher_4in1 = self.create_publisher(FourInOne, '/EL/four_in_one', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.usb_port_BMS = usb_port_BMS
        self.usb_port_4in1 = usb_port_4in1

        self.client = None
        self.bms_available = False

        self.instrument = None
        self.FourinOne_available = False

        self.reconnect_counter = 0
        self.reconnect_interval = 2 
        self.try_connect_BMS()
    
    def try_connect_BMS(self):
        try:
            self.client = ModbusSerialClient(port=self.usb_port, timeout=2, baudrate=115200)
            if self.client.connect():
                self.get_logger().info("Connected to BMS device.")
                self.bms_available = True
            else:
                self.get_logger().warn("Failed to connect to BMS.")
                self.bms_available = False
        except Exception as e:
            self.get_logger().error(f"Exception during connect BMS: {e}")
            self.client = None
            self.bms_available = False

    def try_connect_4in1(self):
        try:
            #Set up instrument
            self.instrument = minimalmodbus.Instrument(self.usb_port_4in1,1,mode=minimalmodbus.MODE_RTU)
            if self.instrument.serial.is_open:
                self.get_logger().info("Connected to 4in1 device.")
                #Make the settings explicit
                self.instrument.serial.baudrate = 9600        # Baud
                self.instrument.serial.bytesize = 8
                self.instrument.serial.parity   = minimalmodbus.serial.PARITY_NONE
                self.instrument.serial.stopbits = 1
                self.instrument.serial.timeout  = 1          # seconds
                
                # Good practice
                self.instrument.close_port_after_each_call = True
                self.instrument.clear_buffers_before_each_transaction = True
                self.FourinOne_available = True
            else:
                self.get_logger().warn("Failed to connect to 4in1 device.")
                self.FourinOne_available = False
        except Exception as e:
            self.get_logger().error(f"Exception during connect 4in1: {e}")
            self.instrument = None
            self.FourinOne_available = False


    def try_reconnect(self):
        self.get_logger().info("Attempting to reconnect to Modbus device...")
        self.try_connect_BMS()

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




import datetime
import numpy as np
import rclpy
import time

from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
import minimalmodbus

from custom_msg.msg import BMS # Import the BMS custom message
from custom_msg.msg import FourInOne

###############################################################################
#################################### BMS ######################################
###############################################################################
# Configure Modbus Serial Client
usb_port_bms = '/dev/ttyUSB0' # TOP RIGHT OF PI !
port_bms = usb_port_bms

client = ModbusSerialClient(
    port=port_bms, timeout=2, baudrate=115200
)

if not client.connect():
    print("Failed to connect to Modbus device.")
    exit(1)

# Set up logging to terminal
# log_file = datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S') + '-log.csv'
num_cells = 7
cells = list(range(1, num_cells + 1))
# print("Timestamp | " + " | ".join([f"Cell {i} Voltage (V)" for i in cells]) + " | Balances | Temperature (C) | Current (A) | Voltage (V) | Status")

###############################################################################
############################## Configure 4in1 #################################
###############################################################################


###############################################################################
################################# Publisher ###################################
###############################################################################
# Create the PythonPublisher node which reads data from serial and sends it on ROS
class PythonPublisher(Node):

    def __init__(self):
        super().__init__('python_publisher') # Super init calls the node class's constructor
        self.publisher_bms = self.create_publisher(BMS, '/EL/bms_topic', 10) # Create a publisher on the topic BMS_topic
        self.publisher_4in1 = self.create_publisher(FourInOne, '/EL/four_in_one', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # BMS message
        v_bat,status,current,voltages = update_bms()
        msg_bms = BMS()
        msg_bms.v_bat = float(v_bat)
        msg_bms.status = status
        msg_bms.voltages = [int(element) for element in voltages]
        msg_bms.current = float(current)
        self.publisher_bms.publish(msg_bms) # Publish the message on the topic

        # 4in1 message
        temperature = instrument.read_register(TEMP_REGISTER, number_of_decimals=1)
        humidity = instrument.read_register(HUM_REGISTER, number_of_decimals=1)
        ec = instrument.read_register(EC_REGISTER)
        ph = instrument.read_register(PH_REGISTER, number_of_decimals=1)
        msg_4in1 = FourInOne()
        msg_4in1.id = 0
        msg_4in1.temperature = round(float(temperature),1)
        msg_4in1.humidity = round(float(humidity),1)
        msg_4in1.conductivity = round(float(ec),0) 
        msg_4in1.ph = round(float(ph),1)
        self.publisher_4in1.publish(msg_4in1)
        
        # # self.get_logger().info(([f"Cell {str(i)} Voltage (V)" for i in voltages]) + " | Current (A) " + str(current) + " | Status" + status + " | v_bat " + str(v_bat))
        # self.get_logger().info(str([f"Cell {i} Voltage (V)" for i in voltages]) + " | Current (A) " + str(current) + " A" + " | Status: " + status + " | v_bat " + str(v_bat) + " V")

def main(args=None):
    rclpy.init(args=args)

    python_pub =PythonPublisher() # Create an instance of the BMSPublisher class

    rclpy.spin(python_pub) # spin the bms publisher node

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    python_pub.destroy_node()
    rclpy.shutdown() # When finished

# Helper function to read registers
def read_registers(client, address, count, slave_id):
    response = client.read_holding_registers(address=address, count=count, slave=slave_id)
    if response.isError():
        raise Exception(f"Error reading registers at address {address}: {response}")
    return response.registers

# Periodic update function
def update_bms():
    try:
        # Get time stamp
        sample_time = datetime.datetime.now()

        # Read individual cell voltages
        voltages = {}
        for i in cells:
            address = i +8
            registers = read_registers(client, address, 1, slave_id=0xAA)
            voltages[i] = registers[0] / 10000 if registers else 0
            #print(registers)

        # Read balance state and encode as binary
        registers = read_registers(client, 51, 16, slave_id=0xAA)
        balance = registers[0] if registers else 0
        balances = '{:016b}'.format(balance)

        # Read pack current and temperature
        registers = read_registers(client, 38, 2, slave_id=0xAA)
        current = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0] if registers else 0)

        registers = read_registers(client, 36, 2, slave_id=0xAA)
        v_bat = float(np.array(registers, dtype=np.uint16).view(dtype=np.float32)[0] if registers else 0)

        registers = read_registers(client, 48, 2, slave_id=0xAA)
        temperature = registers[0] / 10 if registers else None

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

        # print("vbat: ", type(v_bat))
        # Log data to terminal
        # print(f"{sample_time} | " + " | ".join([f"{voltages[i]:.3f}" for i in cells]) +
        #       f" | {balances[:-2]} | {temperature:.1f} | {current:.2f} | {v_bat:.2f}" + f"| {status}")

        # Write data to log file
        # with open(log_file, 'a') as f:
        #     f.write(sample_time.strftime('%Y-%m-%dT%H:%M:%S'))
        #     f.write(', ')
        #     for i in cells:
        #         f.write(f"{voltages[i]}, ")
        #     f.write(f"{balances[:-2]}, {temperature}, {current}, {voltage}\n")

    except Exception as e:
        print(f"Error during update: {e}")
        return (0," unknown",0)
    
    return (v_bat,status,current,voltages)
