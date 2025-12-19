import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ThoughtsPublisher(Node):
    def __init__(self):
        super().__init__('thoughts_publisher')
        
        self.publisher_ = self.create_publisher(
            String, 
            'ros_thoughts', 
            10
        )
        
        # 设置定时器，每2秒发布一次
        timer_period = 2.0  # 2秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # ROS想法列表
        self.thoughts = [
            "ROS2是一个强大的机器人开发框架，让模块化通信变得简单高效。",
            "学习和使用ROS2让我对分布式机器人系统有了更深入的理解。",
            "ROS2的多语言支持（Python、C++、Java等）让开发更加灵活。"
        ]
        
        # 当前想法索引
        self.thought_index = 0
        
        # 计数器
        self.i = 0
        
        self.get_logger().info('发布者2已启动，开始发布ROS想法...')
        self.get_logger().info(f'共有 {len(self.thoughts)} 条想法待发布')
    
    def timer_callback(self):
        # 获取当前想法
        current_thought = self.thoughts[self.thought_index % len(self.thoughts)]
        
        # 创建消息
        msg = String()
        msg.data = current_thought
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 记录发布信息
        self.get_logger().info(f'[发布者2] 第{self.i+1}次发布: {current_thought[:40]}...')
        
        # 更新索引和计数器
        self.thought_index += 1
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    thoughts_publisher = ThoughtsPublisher()
    
    try:
        rclpy.spin(thoughts_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        thoughts_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
