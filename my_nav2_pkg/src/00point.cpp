#include <memory>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h" // 쿼터니언 변환을 위한 헤더 파일임

using namespace std::chrono_literals;

class Nav2PosePublisher : public rclcpp::Node { // ROS2 노드를 상속받아 클래스를 만듦
public:
  Nav2PosePublisher() : Node("nav2_pose_select_node") { // 생성자에서 노드 이름을 설정
    // Qos 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    init_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", qos_profile); // 초기 위치를 발행할 퍼블리셔
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", qos_profile); // 목표 위치를 발행할 퍼블리셔

    timer_ = this->create_wall_timer(1s, std::bind(&Nav2PosePublisher::publish_once, this));
  }

private:
  void publish_once() { // 한 번만 실행될 함수를 정의
    timer_->cancel(); // 타이머를 취소해서 함수가 반복 실행되지 않게 함

    // 초기 위치 메시지를 생성
    auto init_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    init_msg.header.frame_id = "map"; // 기준 좌표계를 맵으로 설정
    init_msg.header.stamp = this->now(); // 현재 시간을 헤더에 입력
    init_msg.pose.pose.position.x = 0.0; // x좌표를 0으로 설정
    init_msg.pose.pose.position.y = 0.0; // y좌표를 0으로 설정
    
    tf2::Quaternion q; // 쿼터니언 변수를 선언
    q.setRPY(0, 0, M_PI); // 오일러 각을 쿼터니언으로 변환
    init_msg.pose.pose.orientation.x = q.x(); // 쿼터니언 x 값
    init_msg.pose.pose.orientation.y = q.y(); // 쿼터니언 y 값
    init_msg.pose.pose.orientation.z = q.z(); // 쿼터니언 x 값
    init_msg.pose.pose.orientation.w = q.w(); // 쿼터니언 w 값
    
    init_msg.pose.covariance[0] = 0.25; // 불확실성 공분산 값을 넣음
    init_msg.pose.covariance[7] = 0.25;
    init_msg.pose.covariance[35] = 0.06;
    init_pub_->publish(init_msg);
    RCLCPP_INFO(this->get_logger(), "Published Initial Pose"); // 로그 출력

    // 목표 위치 메시지를 생성
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position.x = 2.0;
    goal_msg.pose.position.y = 1.0;

    q.setRPY(0, 0, 1.57); // 90도 회전하도록 각도를 설정
    goal_msg.pose.orientation.x = q.x(); 
    goal_msg.pose.orientation.y = q.y(); 
    goal_msg.pose.orientation.z = q.z(); 
    goal_msg.pose.orientation.w = q.w();

    goal_pub_->publish(goal_msg); 
    RCLCPP_INFO(this->get_logger(), "Published Goal Pose"); // 로그 출력
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_; // 초기 위치 퍼블리셔 변수
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_; // 목표 위치 퍼블리셔 변수
  rclcpp::TimerBase::SharedPtr timer_; // 타이머 변수
};

int main(int argc, char **argv) { // 메인 함수 시작
  rclcpp::init(argc, argv); // 초기화
  auto node = std::make_shared<Nav2PosePublisher>(); // 작성한 노드 객체를 생성
  rclcpp::spin(node); // 노드가 작동하도록 스핀
  rclcpp::shutdown(); // 종료
  return 0;
}