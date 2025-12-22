#include "object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
//#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
//#include <iostream>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono;

// 이전 프레임에서 찾은 라인의 위치를 기억하는 변수
// 초기값은 ROI의 중앙(320, 45)으로 설정
cv::Point tmp_pt(320, 45);

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{   
    
    auto start_time = steady_clock::now();
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    // 빈 프레임이 들어올 경우 에러 방지
    if (frame.empty()) return; 

    cv::Rect roi_rect(0, 270, 640, 90);
    cv::Mat roi = frame(roi_rect);

    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    //결과 = 입력 + (목표평균 - 현재평균)
    cv::Scalar avg_pixel = cv::mean(gray);
    //목표 밝기 설정
    double target_mean = 100.0;

    gray = gray+(target_mean-avg_pixel[0]);

    cv::Mat binary;
    cv::threshold(gray, binary, 150, 255, cv::THRESH_BINARY);

    //객체 검출하고 그리기
    cv::Mat stats, centroids;
    findObjects(binary, tmp_pt, stats, centroids);
    drawObjects(stats, centroids, tmp_pt, binary);

    cv::imshow("Video Subscriber", frame);
    cv::imshow("Binary", binary);
    cv::waitKey(1);
    
    auto end_time = steady_clock::now();
    float totalTime = duration<float, std::milli>(end_time - start_time).count(); // 밀리초 단위

 
    int error;
    error = Errorobject(binary, tmp_pt);

    RCLCPP_INFO(node->get_logger(), "error: %d / Time:%f",error,totalTime);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sub_video");
    
  
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP (
    //auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //UDP 
    
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    
    
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}