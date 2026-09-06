#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 模型路径（请根据你的实际情况修改）
        # 这里假设 best.pt 已经放在工作空间根目录
        model_path = os.path.expanduser('~/autodl-tmp/Unknow-gen0_humble/best.pt')
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model not found: {model_path}')
            raise FileNotFoundError(model_path)
        
        self.model = YOLO(model_path)
        
        # 类别名称（与你的 data.yaml 严格一致）
        self.class_names = ['bottle', 'can', 'coffee_cup', 'crushed_can',
                            'food_can', 'paper_crumple', 'small_bottle', 'box']
        
        self.bridge = CvBridge()
        
        # 订阅摄像头话题（注意名称）
        self.sub = self.create_subscription(
            Image,
            '/gen0_model/front_camera',
            self.callback,
            10
        )
        
        # 发布检测结果图像（可选）
        self.pub = self.create_publisher(Image, '/yolo/detected_image', 10)
        
        self.get_logger().info('YOLO node started, waiting for images...')
        
        # 用于保存前几张检测图片
        self.save_count = 0
        self.max_save = 5

    def callback(self, msg):
        try:
            # 转换为 OpenCV 格式
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 运行 YOLO 推理
            results = self.model(cv_img, conf=0.25)
            
            # 绘制检测结果（包含框和名称）
            annotated = results[0].plot()
            
            # 发布带标注的图像
            out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            self.pub.publish(out_msg)
            
            # 保存前几张检测结果到 /tmp 方便查看（无 GUI 环境）
            if self.save_count < self.max_save:
                cv2.imwrite(f'/tmp/detected_{self.save_count:03d}.png', annotated)
                self.get_logger().info(f'Saved /tmp/detected_{self.save_count:03d}.png')
                self.save_count += 1
            
            # 打印检测到的物体信息
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = self.class_names[cls_id] if cls_id < len(self.class_names) else str(cls_id)
                    self.get_logger().info(f'Detected: {name} ({conf:.2f})')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()