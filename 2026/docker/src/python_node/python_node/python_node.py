import datetime
import rclpy
from rclpy.node import Node
import serial
import time
import os
import struct 

from custom_msg.msg import BMS

usb_port_bms = '/dev/ttyESP32_Avionics'


class PythonPublisher(Node):
    def __init__(self):
        super().__init__('python_publisher')
        self.publisher_bms = self.create_publisher(BMS, '/EL/bms_topic', 10)

        self.timer = self.create_timer(1, self.timer_callback) # 1 Hz

        # TinyBMS state
        self.bms_serial = None
        self.bms_available = False
        self.bms_reconnect_counter = 0
        self.bms_reconnect_interval = 2
        self.try_connect_bms()

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

    # ---------------------- Main Timer Callback ---------------------- #
    def timer_callback(self):
        if not self.bms_available:
            self.bms_reconnect_counter += 1
            if self.bms_reconnect_counter >= self.bms_reconnect_interval:
                self.bms_reconnect_counter = 0
                self.try_connect_bms()

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

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    python_pub = PythonPublisher()

    executor.add_node(python_pub)

    executor.spin()
    rclpy.shutdown()
