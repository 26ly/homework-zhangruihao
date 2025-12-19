#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <functional>
#include <algorithm>

using namespace std::chrono_literals;

// A*算法节点结构体
struct AStarNode {
    int x, y;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<AStarNode> parent;
    
    AStarNode(int x, int y, double g, double h, std::shared_ptr<AStarNode> parent = nullptr)
        : x(x), y(y), g_cost(g), h_cost(h), parent(parent) {
        f_cost = g_cost + h_cost;
    }
    struct Compare {
        bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
            return a->f_cost > b->f_cost;
        }
    };
};

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner() : Node("path_planner") {
        // 订阅者
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PathPlanner::map_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PathPlanner::odom_callback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&PathPlanner::goal_callback, this, std::placeholders::_1));
        
        // 发布者
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        
        // 定时器
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PathPlanner::plan_path, this));
        
        RCLCPP_INFO(this->get_logger(), "Path planner node initialized");
    }

private:
    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 数据存储
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    
    bool map_received_ = false;
    bool goal_received_ = false;
    bool odom_received_ = false;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = msg;
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f", 
                   msg->info.width, msg->info.height, msg->info.resolution);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = msg;
        odom_received_ = true;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_goal_ = msg;
        goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", 
                   msg->pose.position.x, msg->pose.position.y);
    }

    // 世界坐标转地图坐标
    bool world_to_map(double wx, double wy, int& mx, int& my) {
        if (!current_map_) return false;
        
        mx = static_cast<int>((wx - current_map_->info.origin.position.x) / current_map_->info.resolution);
        my = static_cast<int>((wy - current_map_->info.origin.position.y) / current_map_->info.resolution);
        
        return mx >= 0 && mx < static_cast<int>(current_map_->info.width) &&
               my >= 0 && my < static_cast<int>(current_map_->info.height);
    }

    // 地图坐标转世界坐标
    void map_to_world(int mx, int my, double& wx, double& wy) {
        if (!current_map_) return;
        
        wx = current_map_->info.origin.position.x + (mx + 0.5) * current_map_->info.resolution;
        wy = current_map_->info.origin.position.y + (my + 0.5) * current_map_->info.resolution;
    }

    // 检查单元格是否可通行
    bool is_cell_free(int x, int y) {
        if (!current_map_ || x < 0 || x >= static_cast<int>(current_map_->info.width) ||
            y < 0 || y >= static_cast<int>(current_map_->info.height)) {
            return false;
        }
        
        int index = y * current_map_->info.width + x;
        return current_map_->data[index] == 0;
    }

    // 计算欧几里得距离
    double heuristic(int x1, int y1, int x2, int y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    // A*路径规划算法
    std::vector<std::pair<int, int>> a_star_search(int start_x, int start_y, int goal_x, int goal_y) {
        if (!current_map_) {
            RCLCPP_WARN(this->get_logger(), "No map available for A* search");
            return {};
        }
        
        const int dx[] = {-1, 0, 1, 0};
        const int dy[] = {0, 1, 0, -1};
        const double cost = 1.0;
        
        // 使用优先队列
        std::priority_queue<std::shared_ptr<AStarNode>, 
                          std::vector<std::shared_ptr<AStarNode>>, 
                          AStarNode::Compare> open_list;
        
        // 关闭列表
        std::vector<std::vector<bool>> closed_list(
            current_map_->info.height, 
            std::vector<bool>(current_map_->info.width, false));
        
        auto start_node = std::make_shared<AStarNode>(
            start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
        open_list.push(start_node);
        
        while (!open_list.empty()) {
            auto current = open_list.top();
            open_list.pop();
            
            if (closed_list[current->y][current->x]) {
                continue;
            }
            closed_list[current->y][current->x] = true;
            
            // 到达目标点
            if (current->x == goal_x && current->y == goal_y) {
                std::vector<std::pair<int, int>> path;
                auto node = current;
                
                while (node != nullptr) {
                    path.push_back(std::make_pair(node->x, node->y));
                    node = node->parent;
                }
                
                std::reverse(path.begin(), path.end());
                RCLCPP_INFO(this->get_logger(), "Path found with %zu points", path.size());
                return path;
            }
            
            // 探索邻居节点
            for (int i = 0; i < 4; ++i) {
                int nx = current->x + dx[i];
                int ny = current->y + dy[i];
                
                if (nx < 0 || nx >= static_cast<int>(current_map_->info.width) ||
                    ny < 0 || ny >= static_cast<int>(current_map_->info.height)) {
                    continue;
                }
                
                if (!is_cell_free(nx, ny) || closed_list[ny][nx]) {
                    continue;
                }
                
                double new_g = current->g_cost + cost;
                double new_h = heuristic(nx, ny, goal_x, goal_y);
                
                auto neighbor = std::make_shared<AStarNode>(nx, ny, new_g, new_h, current);
                open_list.push(neighbor);
            }
        }
        
        RCLCPP_WARN(this->get_logger(), "No path found from (%d, %d) to (%d, %d)", 
                   start_x, start_y, goal_x, goal_y);
        return {}; // 未找到路径
    }

    void plan_path() {
        if (!map_received_ || !goal_received_ || !odom_received_) {
            if (!map_received_) RCLCPP_DEBUG(this->get_logger(), "Waiting for map...");
            if (!goal_received_) RCLCPP_DEBUG(this->get_logger(), "Waiting for goal...");
            if (!odom_received_) RCLCPP_DEBUG(this->get_logger(), "Waiting for odom...");
            return;
        }
        
        // 获取起点（从odom）
        int start_x, start_y;
        if (!world_to_map(current_odom_->pose.pose.position.x, 
                         current_odom_->pose.pose.position.y, 
                         start_x, start_y)) {
            RCLCPP_WARN(this->get_logger(), "Start position outside map: (%.2f, %.2f)", 
                       current_odom_->pose.pose.position.x, current_odom_->pose.pose.position.y);
            return;
        }
        
        // 获取目标点
        int goal_x, goal_y;
        if (!world_to_map(current_goal_->pose.position.x, 
                         current_goal_->pose.position.y, 
                         goal_x, goal_y)) {
            RCLCPP_WARN(this->get_logger(), "Goal position outside map: (%.2f, %.2f)", 
                       current_goal_->pose.position.x, current_goal_->pose.position.y);
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning path from (%d, %d) to (%d, %d)", 
                   start_x, start_y, goal_x, goal_y);
        
        // 检查起点和目标点是否可通行
        if (!is_cell_free(start_x, start_y)) {
            RCLCPP_WARN(this->get_logger(), "Start position (%d, %d) is occupied", start_x, start_y);
            return;
        }
        
        if (!is_cell_free(goal_x, goal_y)) {
            RCLCPP_WARN(this->get_logger(), "Goal position (%d, %d) is occupied", goal_x, goal_y);
            return;
        }
        
        // 执行A*搜索
        auto path_cells = a_star_search(start_x, start_y, goal_x, goal_y);
        
        if (path_cells.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path found");
            return;
        }
        
        // 创建Path消息
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        for (const auto& cell : path_cells) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            
            double wx, wy;
            map_to_world(cell.first, cell.second, wx, wy);
            
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            
            // 设置默认朝向
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path_msg.poses.push_back(pose);
        }
        
        // 发布路径
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Path published with %zu points", path_msg.poses.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
