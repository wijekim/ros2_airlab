#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class Nav2PosePublisher : public rclcpp::Node {
public:
  Nav2PosePublisher() : Node("nav2_pose_select_node") {
    // Transient Local QoS: 노드 종료 후에도 메시지 보존
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    init_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", qos_profile);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", qos_profile);

    // 1초 뒤 실행 (시스템 안정화 대기)
    timer_ = this->create_wall_timer(1s, std::bind(&Nav2PosePublisher::publish_once, this));
  }

private:
  void publish_once() {
    timer_->cancel();

    // 1. Initial Pose 발행
    auto init_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    init_msg.header.frame_id = "map";
    init_msg.header.stamp = this->now();
    init_msg.pose.pose.position.x = 0.0;
    init_msg.pose.pose.position.y = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI);
    init_msg.pose.pose.orientation.x = q.x();
    init_msg.pose.pose.orientation.y = q.y();
    init_msg.pose.pose.orientation.z = q.z();
    init_msg.pose.pose.orientation.w = q.w();
    
    init_msg.pose.covariance[0] = 0.25;
    init_msg.pose.covariance[7] = 0.25;
    init_msg.pose.covariance[35] = 0.06;
    init_pub_->publish(init_msg);
    RCLCPP_INFO(this->get_logger(), "Published Initial Pose");

    // 2. Goal Pose 발행 (약간의 간격을 두고 발행하여 안정성 확보)
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position.x = 2.0;
    goal_msg.pose.position.y = 1.0;

    q.setRPY(0, 0, 1.57);
    goal_msg.pose.orientation.x = q.x();
    goal_msg.pose.orientation.y = q.y();
    goal_msg.pose.orientation.z = q.z();
    goal_msg.pose.orientation.w = q.w();

    goal_pub_->publish(goal_msg);
    RCLCPP_INFO(this->get_logger(), "Published Goal Pose");
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2PosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}