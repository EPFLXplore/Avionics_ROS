import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient

from custom_msg.msg import BMS # Import the BMS custom message

# Configure Modbus Serial Client
usb_port = '/dev/ttyUSB0' # TOP RIGHT OF PI !
port = usb_port


client = ModbusSerialClient(
    port=port, timeout=2, baudrate=115200
)

if not client.connect():
    print("Failed to connect to Modbus device.")
    exit(1)

# Set up logging to terminal
log_file = datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S') + '-log.csv'
num_cells = 7
cells = list(range(1, num_cells + 1))

print("Timestamp | " + " | ".join([f"Cell {i} Voltage (V)" for i in cells]) + " | Balances | Temperature (C) | Current (A) | Voltage (V) | Status")

# Create the BMSPublisher node which reads data from serial and sends it on ROS
class BMSPublisher(Node):

    def __init__(self):
        super().__init__('BMS_publisher') # Super init calls the node class's constructor
        self.publisher_ = self.create_publisher(BMS, 'BMS_topic', 10) # Create a publisher on the topic BMS_topic
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        v_bat,status,current,voltages = update()
        msg = BMS()
        msg.v_bat = float(v_bat)
        msg.status = status
        msg.voltages = [int(element) for element in voltages]
        msg.current = float(current)
        self.publisher_.publish(msg) # Publish the message on the topic
        # self.get_logger().info(([f"Cell {str(i)} Voltage (V)" for i in voltages]) + " | Current (A) " + str(current) + " | Status" + status + " | v_bat " + str(v_bat))
        self.get_logger().info(str([f"Cell {i} Voltage (V)" for i in voltages]) + " | Current (A) " + str(current) + " A" + " | Status: " + status + " | v_bat " + str(v_bat) + " V")


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

        

        print("vbat: ", type(v_bat))
        # Log data to terminal
        print(f"{sample_time} | " + " | ".join([f"{voltages[i]:.3f}" for i in cells]) +
              f" | {balances[:-2]} | {temperature:.1f} | {current:.2f} | {v_bat:.2f}" + f"| {status}")

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


# # Run updates periodically
# if __name__ == "__main__":
#     import time
#     try:
#         while True:
#             update()
#             # print(get_bms_data())
#             time.sleep(5)  # 5-second interval
#     except KeyboardInterrupt:
#         print("Terminating...")
#         client.close()