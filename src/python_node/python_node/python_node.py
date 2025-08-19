import datetime
import rclpy
from rclpy.node import Node
import minimalmodbus
import serial
import time
import os
import struct 

from custom_msg.msg import BMS, FourInOne, LEDMessage, ServoRequest

usb_port_bms = '/dev/ttyBMS' # TOP RIGHT OF PI 
usb_port_4in1 = '/dev/tty4in1' # BOTTOM LEFT
usb_port_leds = '/dev/ttyESP32_LED' # TOP LEFT

# 4in1 register definitions
HUM_REGISTER = 0
TEMP_REGISTER = 1
EC_REGISTER   = 2
PH_REGISTER   = 3

ServoCam_ID = 1
ServoDrill_ID = 2


class PythonPublisher(Node):
    def __init__(self):
        super().__init__('python_publisher')
        self.publisher_bms = self.create_publisher(BMS, '/EL/bms_topic', 10)
        self.publisher_4in1 = self.create_publisher(FourInOne, '/EL/four_in_one', 10)

        self.timer = self.create_timer(1, self.timer_callback) # 1 Hz

        # TinyBMS state
        self.bms_serial = None
        self.bms_available = False
        self.bms_reconnect_counter = 0
        self.bms_reconnect_interval = 2
        self.try_connect_bms()

        # 4in1 Setup
        self.usb_port_4in1 = usb_port_4in1
        self.instrument_4in1 = None
        self.FourinOne_available = False
        self.FourinOne_reconnect_counter = 0
        self.FourinOne_reconnect_interval = 2 
        self.try_connect_4in1()

        # Set servo to init positions for NAV. Two different init positions:
        # 1) when we turn on the rover and the esp32 is launched, sets a position
        # 2) when you launch the avionics docker and code, 2nd position defined below
        # why? 
        # allows us to see clearly if the avionics node is launched.
        publisher_servo = self.create_publisher(ServoRequest, '/EL/servo_req', 10)
        msg_servo_drill = ServoRequest()
        msg_servo_cam = ServoRequest()

        msg_servo_drill.id = ServoDrill_ID
        msg_servo_cam.id = ServoCam_ID

        msg_servo_drill.increment = -1000
        msg_servo_cam.increment = 1000

        msg_servo_drill.zero_in = False
        msg_servo_cam.zero_in = False

        publisher_servo.publish(msg_servo_drill)
        publisher_servo.publish(msg_servo_cam)

    # ---------------------- TinyBMS helpers ---------------------- #
    # quick brief: I had so many issues communicating with the BMS with the 
    # minimalmodbus library that I just went with a custom driver.
    # very similar to the serial used for the avionics and just makes it
    # much more clean.
    @staticmethod
    def crc16(data: bytes) -> int:
        """CRC‑16/Modbus (poly 0x8005, init 0xFFFF, reflected)."""
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _tx_frame(self, cmd: int):
        frame = bytearray([0xAA, cmd])
        crc = self.crc16(frame)
        frame += bytes([crc & 0xFF, crc >> 8])
        self.bms_serial.write(frame)
        self.bms_serial.flush()

    def send_cmd(self, cmd: int, resp_len: int) -> bytes:
        self.bms_serial.reset_input_buffer()

        attempts = 2  # first = normal, second = wake‑up if needed
        for attempt in range(attempts):
            self._tx_frame(cmd)
            start = time.monotonic()
            while time.monotonic() - start < self.bms_serial.timeout:
                hdr = self.bms_serial.read(2)
                if len(hdr) < 2:
                    continue 
                if hdr[0] != 0xAA:
                    continue  # garbage, keep scanning
                payload_cmd = hdr[1]
                # Determine expected total length for this incoming packet
                if payload_cmd in (0x14, 0x15):  
                    total_len = 8
                elif payload_cmd == 0x18:       
                    total_len = 6
                else:
                    continue
                rest = self.bms_serial.read(total_len - 2)
                if len(rest) != total_len - 2:
                    break 
                packet = hdr + rest
                # CRC check
                if self.crc16(packet[:-2]) != (packet[-2] | (packet[-1] << 8)):
                    continue  # bad CRC, ignore
                if payload_cmd == cmd:
                    return packet  # success
        raise TimeoutError(f"TinyBMS: no valid response to 0x{cmd:02X}")

    def read_BMS(self):
        if not self.bms_serial:
            raise RuntimeError("BMS serial not open")

        # Voltage (cmd 0x14)
        v_pkt = self.send_cmd(0x14, 8)
        v_bat = struct.unpack('<f', v_pkt[2:6])[0]

        # Current (cmd 0x15)
        i_pkt = self.send_cmd(0x15, 8)
        pack_current = struct.unpack('<f', i_pkt[2:6])[0]

        # Online status (cmd 0x18)
        s_pkt = self.send_cmd(0x18, 6)
        status_code = s_pkt[2] | (s_pkt[3] << 8)

        status_map = {
            0x91: "Charging",
            0x92: "Fully‑Charged",
            0x93: "Discharging",
            0x96: "Regeneration",
            0x97: "Idle",
            0x9B: "Fault"
        }
        status = status_map.get(status_code, f"0x{status_code:04X}")
        if(status == "Idle"):
            current = 0.0

        return (v_bat, status, pack_current)

    def try_connect_bms(self):
        try:
            self.bms_serial = serial.Serial(
                port=usb_port_bms,
                baudrate=115200,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=1,
                timeout=1
            )
            self.get_logger().info("Connected to TinyBMS serial port.")
            self.bms_available = True
        except Exception as e:
            self.get_logger().warn(f"Failed to open BMS serial port: {e}")
            self.bms_serial = None
            self.bms_available = False

    # ---------------------- 4in1 helpers ---------------------- #
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
        self.get_logger().info("Attempting to reconnect to 4in1 device...")
        self.try_connect_4in1()

    # ---------------------- Main Timer Callback ---------------------- #
    def timer_callback(self):
        if not self.bms_available:
            self.bms_reconnect_counter += 1
            if self.bms_reconnect_counter >= self.bms_reconnect_interval:
                self.bms_reconnect_counter = 0
                self.try_connect_bms()

        if not self.FourinOne_available:
            self.FourinOne_reconnect_counter += 1
            if self.FourinOne_reconnect_counter >= self.FourinOne_reconnect_interval:
                self.FourinOne_reconnect_counter = 0
                self.try_reconnect_4in1()

        # Read & publish BMS
        try:
            v_bat, status, current = self.read_BMS() if self.bms_available else (0.0, "disconnected", 0.0)
        except Exception as e:
            self.get_logger().warn(f"BMS read error: {e}")
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
        msg_4in1.humidity = round(float(humidity),1)
        msg_4in1.conductivity = round(float(ec),0) 
        msg_4in1.ph = round(float(ph),1)
        self.publisher_4in1.publish(msg_4in1)

class PythonSubscriber(Node):
    def __init__(self):
        super().__init__('python_subscriber')
        self.subscription = self.create_subscription(LEDMessage,'/EL/LedCommands',
                                                     self.leds_callback, 10)
        self.subscription 
        self.port = usb_port_leds
        self.serial = None
        self.open_serial_port()
        self.get_logger().info("LED Subscriber node initialized and subscribing to /EL/LedCommands")

        publisher_led = self.create_publisher(LEDMessage, '/EL/LedCommands', 10)
        msg_led = LEDMessage()
        msg_led.system = 0
        msg_led.state = 6
        publisher_led.publish(msg_led)

        msg_led = LEDMessage()
        msg_led.system = 3
        msg_led.state = 1
        publisher_led.publish(msg_led)
   
    def open_serial_port(self):
        try:
            self.serial = serial.Serial(self.port, baudrate=115200, timeout=1)
            self.get_logger().info(f"Serial port {self.port} opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.serial = None

    def leds_callback(self, msg):
        # self.get_logger().info('Received LED message:')
        # self.get_logger().info('System "%s"' % msg.system)
        # self.get_logger().info('State "%s"' % msg.state)

        if self.serial != None:
            try:
                mode = 6
                # Prepare the LED message
                if msg.system not in [0, 1, 2, 3]:
                    self.get_logger().error("Invalid system value. Must be 0, 1, 2, or 3.")
                    return
                if msg.state not in [0, 1, 2, 3, 4, 5, 6]:
                    self.get_logger().error("Invalid state value. Must be 0, 1, or 2.")
                    return
                if msg.state == 0:
                    mode = 0  # Off
                    # self.get_logger().info("Turning off LEDs.")
                elif msg.state == 1:
                    mode = 1  # On
                    # self.get_logger().info("Turning on LEDs.")
                elif msg.state == 2:
                    mode = 2  # Blinking
                    # self.get_logger().info("Blinking LEDs.")
                elif msg.state == 3:
                    mode = 3  # Fault
                    # self.get_logger().info("Blinking LEDs.")
                elif msg.state == 4:
                    mode = 4  # Emergency Motors
                    # self.get_logger().info("Blinking LEDs.")
                elif msg.state == 5:
                    mode = 5  # Emergency Shutdown
                    # self.get_logger().info("Blinking LEDs.")
                elif msg.state == 6:
                    mode = 6  # All off
                    # self.get_logger().info("Blinking LEDs.")

                led_message = f"0 1 {msg.system} {mode}\n"
                self.serial.write(led_message.encode('ascii'))
                # self.get_logger().info("LED message sent successfully.")
                
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")
        else:
            self.get_logger().error("Serial port is not open. Cannot send LED message.")
            self.open_serial_port()

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    python_pub = PythonPublisher() 
    python_sub = PythonSubscriber()

    executor.add_node(python_pub)
    executor.add_node(python_sub)

    executor.spin()
    rclpy.shutdown()
