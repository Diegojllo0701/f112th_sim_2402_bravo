import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.subscription = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, '/angles', 10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg: LaserScan):
        angles_to_check = [0, 90, 180, 270]  # angles in degrees
        distances = self.get_distances_at_angles(msg, angles_to_check)
        for angle, distance in distances.items():
            self.get_logger().info(f'Distance at {angle} degrees: {distance:.2f} meters')
        
        angles_array = Float32MultiArray(data=angles_to_check)
        self.publisher.publish(angles_array)

    def get_distances_at_angles(self, msg, angles):
        distances = {}
        for angle in angles:
            index = int((angle - msg.angle_min) / msg.angle_increment)
            distances[angle] = msg.ranges[index]
        return distances

def main(args=None):
    rclpy.init(args=args)
    scan_reader = ScanReader()
    rclpy.spin(scan_reader)
    scan_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
