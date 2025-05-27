import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import struct
import threading

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.create_subscription(Point, 'botani/coordinates', self.coordinates_callback, 10)

        # Start a background thread for reading the serial data from Arduino
        thread = threading.Thread(target=self.read_serial_loop)
        thread.daemon = True
        thread.start()

    def coordinates_callback(self, msg):
        x = msg.x
        y = msg.y
        # Pack the data into binary format (2 floats for x and y)
        data = struct.pack('ff', x, y)
        self.ser.write(data)  # Send packed data to Arduino

        self.get_logger().info(f"Sent to Arduino: x={x}, y={y}")

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                # Read the feedback from Arduino (e.g., "yes" or "no")
                data = self.ser.readline().decode('utf-8').strip()  # Read a line and decode it
                if data in ["yes", "no"]:
                    self.get_logger().info(f"From Arduino: Task finished? {data}")
                else:
                    self.get_logger().warn(f"Unexpected feedback from Arduino: {data}")
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()