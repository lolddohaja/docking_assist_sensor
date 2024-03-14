import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

class CenterDetectionNode(Node):
    def __init__(self):
        super().__init__('docking_guide')
        self.declare_parameter('left_right_threshold', 0.1)
        self.declare_parameter('front_threshold', 0.5)

        self.sensor_subscription = self.create_subscription(Float32MultiArray, 'sensor_data', self.handle_sensor_data, 10)

        self.center_status_publisher = self.create_publisher(Bool, 'center_status', 10)

        self.left_distance = 0.0
        self.right_distance = 0.0
        self.front_distance = 0.0


    def handle_sensor_data(self, msg):
        # 센서 데이터 배열에서 각 거리 값을 할당
        self.left_distance = msg.data[0]
        self.right_distance = msg.data[1]
        self.front_distance = msg.data[2]
        self.evaluate_position()

    def evaluate_position(self):
        left_right_threshold = self.get_parameter('left_right_threshold').get_value()
        front_threshold = self.get_parameter('front_threshold').get_value()

        if abs(self.left_distance - self.right_distance) < left_right_threshold and self.front_distance < front_threshold:
            self.get_logger().info('Robot is approximately at the center.')
            self.center_status_publisher.publish(Bool(data=True))
        else:
            self.get_logger().info('Robot is not at the center.')
            self.center_status_publisher.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = CenterDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
