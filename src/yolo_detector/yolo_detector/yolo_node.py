#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import os
import traceback
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 模型路径（请根据你的实际情况修改）
        # 这里假设 best.pt 已经放在工作空间根目录
        self.declare_parameter('model_path', '"D:\_desktop\D_desktopxw6Rio9mOoSR\best_road.pt"')
        self.declare_parameter('confidence', 0.50)
        self.declare_parameter('output_dir', '/root/autodl-tmp/yolo_trials/current')
        self.declare_parameter('save_every_n', 10)
        model_path = os.path.expanduser(self.get_parameter('model_path').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.save_every_n = max(1, int(self.get_parameter('save_every_n').value))
        os.makedirs(self.output_dir, exist_ok=True)
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model not found: {model_path}')
            raise FileNotFoundError(model_path)
        
        self.model = YOLO(model_path)
        
        # 类别名称（与你的 data.yaml 严格一致）
        self.class_names = self.model.names
        
        self.bridge = CvBridge()
        
        # 订阅摄像头话题（注意名称）
        self.sub = self.create_subscription(
            Image,
            '/gen0_model/front_camera',
            self.callback,
            qos_profile_sensor_data
        )
        
        # 发布检测结果图像（可选）
        self.pub = self.create_publisher(Image, '/yolo/detected_image', 10)
        
        self.get_logger().info(
            f'YOLO node started: model={model_path}, conf={self.confidence:.2f}, '
            f'output={self.output_dir}'
        )
        
        # 用于保存前几张检测图片
        self.frame_count = 0
        self.detection_frame_count = 0

    def callback(self, msg):
        try:
            # 转换为 OpenCV 格式
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 运行 YOLO 推理
            self.frame_count += 1
            results = self.model(cv_img, conf=self.confidence, verbose=False)
            
            # 绘制检测结果（包含框和名称）
            annotated = results[0].plot()
            
            # 发布带标注的图像
            # cv_bridge from ROS Humble can raise KeyError(16) with newer NumPy
            # when converting an OpenCV array back to a message. Build the
            # canonical bgr8 Image directly so the annotated stream remains
            # portable across the container's mixed Python packages.
            annotated = cv2.convertScaleAbs(annotated)
            out_msg = Image()
            out_msg.header = msg.header
            out_msg.height = int(annotated.shape[0])
            out_msg.width = int(annotated.shape[1])
            out_msg.encoding = 'bgr8'
            out_msg.is_bigendian = 0
            out_msg.step = int(annotated.shape[1] * 3)
            out_msg.data = annotated.tobytes()
            self.pub.publish(out_msg)
            
            # 打印检测到的物体信息
            boxes = results[0].boxes
            count = len(boxes) if boxes is not None else 0
            if count:
                self.detection_frame_count += 1
            should_save = count > 0 or self.frame_count % self.save_every_n == 0
            if should_save:
                path = os.path.join(
                    self.output_dir,
                    f'frame_{self.frame_count:05d}_detections_{count}.jpg'
                )
                cv2.imwrite(path, annotated)
            self.get_logger().info(
                f'FRAME frame={self.frame_count} detections={count} '
                f'detection_frames={self.detection_frame_count}'
            )
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = self.class_names[cls_id] if cls_id < len(self.class_names) else str(cls_id)
                    self.get_logger().info(f'Detected: {name} ({conf:.2f})')
                    
        except Exception as e:
            self.get_logger().error(
                f'Error processing image: {e!r}\n{traceback.format_exc()}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
