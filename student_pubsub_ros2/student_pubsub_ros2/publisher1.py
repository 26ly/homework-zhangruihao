import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class StudentPublisher(Node):
    def __init__(self):
        super().__init__('student_publisher')
       
        self.publisher_ = self.create_publisher(
            String, 
            'student_info', 
            10
        )
        
        # 设置定时器，每秒发布一次
        timer_period = 1.0  # 1秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 学生信息（使用JSON格式）
        self.student_data = {
            'name': '张睿浩',
            'college': '人工智能与自动化学院'
        }
        
        # 选择格式类型
        self.format_type = 'json'
        
        # 计数器
        self.i = 0
        
        self.get_logger().info('发布者1已启动，开始发布学生信息...')
        self.get_logger().info(f'学生姓名: {self.student_data["name"]}')
        self.get_logger().info(f'所属学院: {self.student_data["college"]}')
        self.get_logger().info(f'使用格式: {self.format_type}')
    
    def timer_callback(self):
        # 创建消息
        msg = String()
        msg.data = json.dumps(self.student_data, ensure_ascii=False)
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 记录发布信息
        self.get_logger().info(f'[发布者1] 第{self.i+1}次发布(JSON): {self.student_data["name"]} - {self.student_data["college"]}')
        
        # 增加计数器
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    student_publisher = StudentPublisher()
    
    try:
        rclpy.spin(student_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        student_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
