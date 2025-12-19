import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class MultiTopicSubscriber(Node):
    def __init__(self):
        super().__init__('multi_topic_subscriber')
        
        # 订阅学生信息话题
        self.student_subscription = self.create_subscription(
            String,
            'student_info',
            self.student_callback,
            10
        )
        
        # 订阅ROS想法话题
        self.thoughts_subscription = self.create_subscription(
            String,
            'ros_thoughts',
            self.thoughts_callback,
            10
        )
        
        # 计数器
        self.student_count = 0
        self.thoughts_count = 0
        
        # 分隔线
        self.separator = "=" * 50
        
        self.get_logger().info('订阅者已启动，正在监听...')
        self.get_logger().info('等待接收消息...')
        self.get_logger().info('学生信息格式: JSON ')
    
    def student_callback(self, msg):
        """处理接收到的学生信息"""
        self.student_count += 1
        
        try:
            # 尝试解析JSON格式
            student_data = json.loads(msg.data)
            name = student_data.get('name', '未知')
            college = student_data.get('college', '未知')
            student_id = student_data.get('student_id', '未知')
            
            print(f"\n{self.separator}")
            print("📋 收到学生信息 (JSON格式):")
            print(f"   消息序号: #{self.student_count}")
            print(f"   学生姓名: {name}")
            print(f"   所属学院: {college}")
            print(f"   接收时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
            print(f"{self.separator}")

        except Exception as e:
                # 如果无法解析，直接显示原始数据
                print(f"\n{self.separator}")
                print("📋 收到学生信息 (原始格式):")
                print(f"   消息序号: #{self.student_count}")
                print(f"   原始数据: {msg.data}")
                print(f"   接收时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
                print(f"   解析错误: {e}")
                print(f"{self.separator}")
    
    def thoughts_callback(self, msg):
        """处理接收到的ROS想法"""
        self.thoughts_count += 1
        
        # 简单处理，直接显示内容
        thought_content = msg.data
        
        print(f"\n{self.separator}")
        print("💭 收到ROS想法:")
        print(f"   消息序号: #{self.thoughts_count}")
        print(f"   内容: {thought_content}")
        print(f"   接收时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
        print(f"   内容长度: {len(thought_content)} 字符")
        print(f"{self.separator}")

def main(args=None):
    rclpy.init(args=args)
    
    multi_topic_subscriber = MultiTopicSubscriber()
    
    try:
        rclpy.spin(multi_topic_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        multi_topic_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
