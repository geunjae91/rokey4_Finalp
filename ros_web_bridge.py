import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import threading
import time

# 웹에서 접근할 전역 변수 (실제 애플리케이션에서는 큐/락 등을 사용하여 안전하게 관리)
global_image_data = None # Base64 인코딩된 이미지 데이터
global_result_data = "Waiting for YOLO results..." # 결과 텍스트

class RosWebBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_web_bridge')
        self.bridge = CvBridge()

        # 이미지 토픽 구독
        self.image_subscription = self.create_subscription(
            Image,
            'yolo_slot_image', # SlotCheckerNode에서 발행하는 이미지 토픽명
            self.image_callback,
            10
        )
        self.get_logger().info('Subscribing to /yolo_slot_image topic')

        # 결과 토픽 구독
        self.result_subscription = self.create_subscription(
            String,
            'yolo_slot_result', # SlotCheckerNode에서 발행하는 결과 토픽명
            self.result_callback,
            10
        )
        self.get_logger().info('Subscribing to /yolo_slot_result topic')

    def image_callback(self, msg):
        global global_image_data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 이미지를 JPEG 형식으로 인코딩하고 Base64로 변환 (웹 전송용)
            ret, buffer = cv2.imencode('.jpg', cv_image)
            if ret:
                global_image_data = base64.b64encode(buffer).decode('utf-8')
            else:
                self.get_logger().warning('Failed to encode image to JPEG.')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def result_callback(self, msg):
        global global_result_data
        global_result_data = msg.data
        self.get_logger().info(f'Received result: {global_result_data[:50]}...') # 너무 길면 자름

# Flask 앱과 ROS 2 노드를 동시에 실행하기 위한 스레딩
def run_ros_node():
    rclpy.init(args=None)
    node = RosWebBridgeNode()
    rclpy.spin(node) # ROS 2 노드가 계속 실행되면서 토픽을 구독
    node.destroy_node()
    rclpy.shutdown()

# --- 아래는 기존 Flask 코드에 추가될 내용 ---
# 현재 Flask 앱 (app.py 또는 main.py) 파일에 이 내용을 통합해야 합니다.
# Flask 서버에서 ROS 2 데이터를 제공하는 새로운 라우트 추가
# @app.route('/get_ros_data')
# def get_ros_data():
#     global global_image_data, global_result_data
#     return {
#         'image': global_image_data,
#         'result': global_result_data
#     }

# if __name__ == '__main__':
#     # Flask 앱 시작 전에 ROS 2 노드를 별도의 스레드에서 시작
#     ros_thread = threading.Thread(target=run_ros_node)
#     ros_thread.daemon = True # 메인 스레드(Flask) 종료 시 함께 종료
#     ros_thread.start()

#     app.run(debug=True, port=5001)