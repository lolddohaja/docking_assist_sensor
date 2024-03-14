import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import bluetooth
import threading

class CenterDetectionNode(Node):
    def __init__(self):
        super().__init__('docking_sensor')

        # ROS2 퍼블리셔 선언
        self.sensor_pub = self.create_publisher(Float32MultiArray, 'sensor_data', 10)

        # 블루투스 장치 스캔 및 연결
        self.scan_and_connect_bluetooth()



    def scan_and_connect_bluetooth(self):
        # 스레드에서 블루투스 연결 및 데이터 수신 작업 실행
        threading.Thread(target=self.bluetooth_connection_thread, daemon=True).start()

    def bluetooth_connection_thread(self):
        # 블루투스 장치 스캔 및 연결 로직
        # 블루투스 장치로부터 데이터를 지속적으로 수신하는 로직
        pass

    def receive_and_publish_sensor_data(self):
        # 여기에 블루투스 장치로부터 데이터를 수신하고 처리하는 코드를 작성합니다.
        # 예제 코드는 실제 데이터 수신 방법을 단순화하여 설명합니다.

        # 데이터 수신 가정
        left_distance = 0.5  # 예시 데이터
        right_distance = 0.5
        back_distance = 1.0

        # Float32MultiArray 메시지 생성 및 데이터 할당

        sensor_data_msg = Float32MultiArray()
        sensor_data_msg.data = [left_distance, right_distance, back_distance]
        
        # 토픽으로 데이터 발행
        self.sensor_pub.publish(sensor_data_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CenterDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
