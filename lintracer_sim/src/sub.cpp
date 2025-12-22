#include "object.hpp"
#include "rclcpp/rclcpp.hpp" // ROS 2 기본 헤더
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp" // OpenCV 헤더
#include <memory> 
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#define STDIN_FILENO 0 

using std::placeholders::_1;
using namespace std::chrono;

// 이전 프레임에서 찾은 라인의 위치를 기억하는 변수
// 초기값은 ROI의 중앙(320, 45)으로 설정
cv::Point tmp_pt(320, 45);

int getch(void) // 엔터키 입력 없이 한 문자만 입력시 바로 리턴
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool kbhit(void) // 키보드가 눌렸는지 체크해주는 함수
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

static bool mode = false;
static int ch = 0;
void mysub_callback(rclcpp::Node::SharedPtr node,rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub,const sensor_msgs::msg::CompressedImage::SharedPtr msg)
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

    float leftvel;
    float rifhtvel;
    double k = 0.2;
    
    leftvel = 100 - k* error;
    rifhtvel = -(100+k*error);

    geometry_msgs::msg::Vector3 vel;

    vel.x = leftvel;
    vel.y = rifhtvel;

    if(kbhit()){
        ch = getch();
        if(ch=='q'){
            mode = false;
            vel.x=0;
            vel.y=0;
        }
        else if(ch == 's'){
            mode = true;
        }
    }

   
    
    mypub->publish(vel);

    RCLCPP_INFO(node->get_logger(), "error: %d / Time:%.2f / Lvel:%.2f / Rvel:%.2f",error,totalTime,leftvel,rifhtvel);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sub_video");
    
  
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP (
    //auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //UDP 
    

    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback,node,mypub, _1);
    
    
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
