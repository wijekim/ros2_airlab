#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>

//1
std::string src = "./simulation/5_lt_cw_100rpm_out.mp4"; 
//2
//std::string src = "./simulation/7_lt_ccw_100rpm_in.mp4"; 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    //노드 생성
    auto node = std::make_shared<rclcpp::Node>("pub_Video");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP
    //auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //UDP

    //Publisher 생성
    //토픽이름 image/compressed
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    
    //주파수 1초40번
    rclcpp::WallRate loop_rate(40.0);

    //동영상 파일 열기
    cv::VideoCapture cap(src);
    
    //영상 끝 루프 종료
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open video file: %s", src.c_str());
        rclcpp::shutdown();
        return -1;
    }
    cv::Mat frame;

    while(rclcpp::ok())
    {
        cap >> frame;
        if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "Video finished or frame empty"); break;}
        
        //이미지(Mat)를 -> ROS 메시지(CompressedImage)로 변환
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();

        //메시지 발행
        mypub->publish(*msg);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}