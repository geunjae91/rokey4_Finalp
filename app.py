from flask import Flask, render_template, request, jsonify
import sqlite3
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import threading
import re
from datetime import datetime

app = Flask(__name__)
DB_PATH = os.path.join(os.path.dirname(__file__), 'database', 'database.db')

# ----- 전역 변수 -----
global_ros_image_data = None
global_ros_result_text = "YOLO 결과 대기 중..."

# ----- ROS 2 노드: 웹 연동 -----
class RosWebBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_web_bridge_node')
        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image, 'yolo_slot_image', self.image_callback, 10)
        self.result_subscription = self.create_subscription(
            String, 'yolo_slot_result', self.result_callback, 10)

        self.get_logger().info('/yolo_slot_image 토픽을 구독')
        self.get_logger().info('/yolo_slot_result 토픽을 구독')

    def image_callback(self, msg):
        global global_ros_image_data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            ret, buffer = cv2.imencode('.jpg', cv_image)
            if ret:
                global_ros_image_data = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')

    def result_callback(self, msg):
        global global_ros_result_text
        global_ros_result_text = msg.data

# ----- YOLO 결과 로깅 노드 -----
class YoloResultLogger(Node):
    def __init__(self):
        super().__init__('yolo_result_logger')
        self.latest_result = "초기화"
        self.subscription = self.create_subscription(
            String, 'yolo_slot_result', self.result_callback, 10)
        self.timer = self.create_timer(10.0, self.save_to_db)
        self.get_logger().info('YOLO 토픽 구독 시작')

    def result_callback(self, msg):
        self.latest_result = msg.data

    def save_to_db(self):
        parsed = self.parse_yolo_result_text(self.latest_result)
        if parsed:
            self.update_slot_status(parsed)
            self.get_logger().info('DB 업데이트 완료')
        else:
            self.get_logger().warn('파싱 실패 또는 빈 결과')

    def parse_yolo_result_text(self, text):
        parsed = {}
        lines = text.strip().split('\n')
        pattern = re.compile(r'Slot (\d+): (.+)\[(.+)\]')
        for line in lines:
            m = pattern.match(line.strip())
            if m:
                slot_id = int(m.group(1))
                content = m.group(2).strip()
                status = m.group(3).strip()
                parsed[slot_id] = (content, status)
        return parsed

    def update_slot_status(self, parsed_data):
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS yolo_slot_status (
                slot_id INTEGER PRIMARY KEY,
                object_name TEXT,
                quantity INTEGER,
                status TEXT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')

        for slot_id, (content, status) in parsed_data.items():
            if content:
                parts = content.split()
                object_name = parts[0] if len(parts) > 0 else ''
                try:
                    quantity = int(parts[1].replace('개', '')) if len(parts) > 1 else 0
                except ValueError:
                    quantity = 0
            else:
                object_name = ''
                quantity = 0

            cursor.execute('''
                INSERT OR REPLACE INTO yolo_slot_status (slot_id, object_name, quantity, status, timestamp)
                VALUES (?, ?, ?, ?, CURRENT_TIMESTAMP)
            ''', (slot_id, object_name, quantity, status))
        conn.commit()
        conn.close()

# ----- ROS 노드 실행 -----
def run_ros_nodes():
    rclpy.init()
    web_node = RosWebBridgeNode()
    yolo_node = YoloResultLogger()
    executor = MultiThreadedExecutor()
    executor.add_node(web_node)
    executor.add_node(yolo_node)
    try:
        executor.spin()
    finally:
        web_node.destroy_node()
        yolo_node.destroy_node()
        rclpy.shutdown()

# ----- Flask API -----
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/login')
def login():
    return render_template('login.html')

@app.route('/order')
def order():
    return render_template('order.html')

@app.route('/admin')
def admin_dashboard():
    return render_template('admin.html')

@app.route('/login_check', methods=['POST'])
def login_check():
    user_id = request.form['id']
    password = request.form['pw']
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM users WHERE id = ? AND password = ?", (user_id, password))
    user = cursor.fetchone()
    conn.close()
    return user_id if user else "fail"

@app.route('/submit_order', methods=['POST'])
def submit_order():
    name = request.form['name']
    address = request.form['address']
    phone = request.form['phone']
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS orders (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT,
            address TEXT,
            phone TEXT
        )
    ''')
    cursor.execute("INSERT INTO orders (name, address, phone) VALUES (?, ?, ?)", (name, address, phone))
    conn.commit()
    conn.close()
    return "success"

@app.route('/add_to_cart', methods=['POST'])
def add_to_cart():
    data = request.get_json()
    product_name = data['product_name']
    quantity = int(data['quantity'])
    if quantity > 1:
        return jsonify(success=False), 400
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT quantity FROM cart_items WHERE product_name = ?", (product_name,))
    row = cursor.fetchone()
    if row:
        cursor.execute("UPDATE cart_items SET quantity = quantity + ? WHERE product_name = ?", (quantity, product_name))
    else:
        cursor.execute("INSERT INTO cart_items (product_name, quantity) VALUES (?, ?)", (product_name, quantity))
    conn.commit()
    conn.close()
    return jsonify(success=True)

@app.route('/cart_items')
def cart_items():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        SELECT ci.product_name, ci.quantity, p.price 
        FROM cart_items ci
        JOIN products p ON ci.product_name = p.product_name
    """)
    items = cursor.fetchall()
    conn.close()
    return jsonify([
        {'product_name': row[0], 'quantity': row[1], 'price': row[2]}
        for row in items
    ])

@app.route('/get_ros_data')
def get_ros_data():
    global global_ros_image_data, global_ros_result_text
    return jsonify({
        'image': global_ros_image_data,
        'result': global_ros_result_text
    })

@app.route('/get_yolo_status')
def get_yolo_status():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('SELECT slot_id, object_name, quantity, status FROM yolo_slot_status')
    rows = cursor.fetchall()
    conn.close()
    return jsonify([
        {'slot_id': row[0], 'object_name': row[1], 'quantity': row[2], 'status': row[3]}
        for row in rows
    ])

# ----- 앱 실행 -----
if __name__ == '__main__':
    ros_thread = threading.Thread(target=run_ros_nodes, daemon=True)
    ros_thread.start()
    app.run(host='0.0.0.0', debug=True, port=5001)
