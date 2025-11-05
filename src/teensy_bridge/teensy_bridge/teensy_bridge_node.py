import rclpy
from rclpy.node import Node
from std_msgs.msg import Float3232
import serial
import serial.tools.list_ports

class TeensyBridge(Node):
    def __init__(self):
        super().__init__('teensy_bridge')

        ports = list(serial.tools.list_ports.comports())
        teensy_port = None
        for p in ports:
            if "Teensy" in p.description or "ACM" in p.device:
                teensy_port = p.device
                break

        if teensy_port is None:
            self.get_logger().error("No Teensy detected. Please plug it in via USB.")
            raise RuntimeError("Teensy not found")

        self.get_logger().info(f"Connected to Teensy on {teensy_port}")

        self.ser = serial.Serial(teensy_port, 9600)
        self.publisher = self.create_publisher(Float32, 'soil_moisture', 10)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.isdigit():
                msg = Float32()
                msg.data = float(line)
                self.publisher.publish(msg)
                self.get_logger().debug(f"Published: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

def main():
    rclpy.init()
    try:
        node = TeensyBridge()
        rclpy.spin(node)
    except Exception as e:
        print(f"[ERROR] Teensy Bridge failed to start: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
