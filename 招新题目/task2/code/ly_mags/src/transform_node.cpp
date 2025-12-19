#include "rclcpp/rclcpp.hpp"
#include "ly_mags/msg/input.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

// 定义PI常量用于角度转换
const double PI = 3.14159265358979323846;

class TransformNode : public rclcpp::Node
{
public:
    TransformNode() : Node("transform_node")
    {
        RCLCPP_INFO(this->get_logger(), "transform_node 节点启动成功");
        // 创建订阅者，订阅/ly/input话题，消息类型为ly_mags::msg::Input
        subscription_ = this->create_subscription<ly_mags::msg::Input>(
            "/ly/input", 10,
            std::bind(&TransformNode::input_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "已订阅话题 /ly/input");
        // 创建四个发布者，分别发布到四个话题
        publisher_x1_ = this->create_publisher<std_msgs::msg::Float32>("/ly/output_x1", 10);
        publisher_y1_ = this->create_publisher<std_msgs::msg::Float32>("/ly/output_y1", 10);
        publisher_x2_ = this->create_publisher<std_msgs::msg::Float32>("/ly/output_x2", 10);
        publisher_y2_ = this->create_publisher<std_msgs::msg::Float32>("/ly/output_y2", 10);
        RCLCPP_INFO(this->get_logger(), "已创建4个发布者");
        RCLCPP_INFO(this->get_logger(), "节点初始化完成，等待消息...");
    }

private:
    void input_callback(const ly_mags::msg::Input::SharedPtr msg)
    {
        // 提取输入数据
        double x = msg->x;
        double y = msg->y;
        double theta_deg = msg->theta;
        double tx = msg->tx;
        double ty = msg->ty;

        // 将角度从度转换为弧度
        double theta_rad = theta_deg * PI / 180.0;

        // 计算旋转矩阵的三角函数值
        double cos_theta = cos(theta_rad);
        double sin_theta = sin(theta_rad);

        // 第一次变换：先旋转后平移
        double x1 = cos_theta * x - sin_theta * y + tx;
        double y1 = sin_theta * x + cos_theta * y + ty;

        // 第二次变换：先平移后旋转
        double x_temp = x + tx;
        double y_temp = y + ty;
        double x2 = cos_theta * x_temp - sin_theta * y_temp;
        double y2 = sin_theta * x_temp + cos_theta * y_temp;

        // 创建并发布消息
        auto msg_x1 = std_msgs::msg::Float32();
        msg_x1.data = x1;
        publisher_x1_->publish(msg_x1);

        auto msg_y1 = std_msgs::msg::Float32();
        msg_y1.data = y1;
        publisher_y1_->publish(msg_y1);

        auto msg_x2 = std_msgs::msg::Float32();
        msg_x2.data = x2;
        publisher_x2_->publish(msg_x2);

        auto msg_y2 = std_msgs::msg::Float32();
        msg_y2.data = y2;
        publisher_y2_->publish(msg_y2);
    }

    rclcpp::Subscription<ly_mags::msg::Input>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_x1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_y1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_x2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_y2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformNode>());
    rclcpp::shutdown();
    return 0;
}