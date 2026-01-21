#include <memory>
#include <vector>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "tf2/LinearMath/Quaternion.h"

class WaypointFollower : public rclcpp::Node {
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  WaypointFollower() : Node("lab_waypoint_follower") {
    this->client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  }

  void send_waypoints() {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not found.");
      return;
    }

    auto goal_msg = FollowWaypoints::Goal();
    std::vector<std::vector<double>> coords = {
        {-1.0035, -0.0055, -3.13}, // wp1: 왼쪽으로 이동
        {-2.0186, -0.0124, -3.14}, // wp2
        {-3.0097, -0.0313,  3.10}, // wp3
        {-3.3849, -1.0206, -1.55}, // wp4: 아래로 회전
        {-3.3422, -2.0358, -1.57}, // wp5
        {-3.0171, -3.0345, -0.01}, // wp6: 오른쪽으로 회전
        {-1.9916, -2.6405, -0.01}, // wp7
        {-1.0289, -2.5122,  0.01}, // wp8
        { 0.3734, -2.4988,  1.54}, // wp9: 위로 회전
        { 0.3909, -1.0455,  1.55}, // wp10
        { 0.0,     0.0,     M_PI}  // 최종 원점 복귀
    };

    for (const auto& coord : coords) { // 좌표 리스트 하나씩 꺼내면서 반복
      geometry_msgs::msg::PoseStamped pose; // 자세 정보 담을 객체
      pose.header.frame_id = "map"; // 맵 기준
      pose.header.stamp = this->now(); 
      pose.pose.position.x = coord[0]; // x값 
      pose.pose.position.y = coord[1]; // y값

      // Yaw 값을 쿼터니언으로 변환하여 설정
      tf2::Quaternion q;
      q.setRPY(0, 0, coord[2]);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      goal_msg.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "총 %zu개의 경유점으로 순환 주행 시작", goal_msg.poses.size());

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&WaypointFollower::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;

  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "원점에 도착함.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "xxxxxxxx");
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollower>();
  node->send_waypoints();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


 // {x, y, yaw} 순서
 /*std::vector<std::vector<double>> coords = {
   {-0.97897,-0.011672,0.2},
   {-2.0403,-0.0063621,0.2},
   {-3.3538,-0.020806,0.2},
   {-3.3538, -1.9751,0.2},
   {-3.0431,-3.0172,0.2},
   {-1.0126,-2.5334,0.2},
   {0.3854,-2.4459,0.2},
   { 0.0,  0.0,    3.14} // 시작할 Pose에서 설정한 PI(3.14)와 동일하게 설정
 };*/

/*std::vector<std::vector<double>> coords = {
   {-0.97897, -0.011672,  M_PI},  // WP1: 계속 왼쪽으로 진행
   {-2.0403,  -0.006362,  M_PI},  // WP2: 계속 왼쪽으로 진행
   {-3.3538,  -0.020806, -1.57}, // WP3: 아래로 꺾기 위해 방향 전환
   {-3.3538,  -1.9751,   -1.8},  // WP4: 대각선 아래 진행
   {-3.0431,  -3.0172,   0.0},   // WP5: 오른쪽으로 진행 시작
   {-1.0126,  -2.5334,   0.0},   // WP6: 계속 오른쪽 진행
   {0.3854,   -2.4459,   1.57},  // WP7: 원점(위쪽)으로 가기 위해 방향 전환
   {0.0,       0.0,      M_PI}   // WP8: 최종 원점 도착 및 시작 자세 유지
 };*/