#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

cv::Point tmp_pt(320, 45);

void findObjects(cv::Mat& thresh, cv::Point& tmp_pt, cv::Mat& stats, cv::Mat& centroids) {
    cv::Mat labels;
    int cnt = cv::connectedComponentsWithStats(thresh, labels, stats, centroids);

    // 1채널(Gray) -> 3채널(BGR) 변환 (컬러로 박스를 그리기 위함)
    cv::cvtColor(thresh, thresh, cv::COLOR_GRAY2BGR);

    int min_index = -1;
    int min_dist = thresh.cols; // 초기 거리값은 크게 설정

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4); // 면적

        if (area > 100) { // 노이즈 제거 (면적 100 이하 무시)
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            
            // 거리 계산
            int dist = cv::norm(cv::Point(x, y) - tmp_pt);    

            // 가장 가깝고 150 픽셀 이내인 객체 선정
            if (dist < min_dist && dist <= 150) {   
                min_dist = dist;
                min_index = i;
            }
        }
    }

    // 객체를 찾았으면 위치 갱신
    if (min_index != -1 && min_dist <= 150) { 
        tmp_pt = cv::Point(cvRound(centroids.at<double>(min_index, 0)), cvRound(centroids.at<double>(min_index, 1)));    
    }
    else {
        // 못 찾았으면 마지막 위치에 점 찍기
        cv::circle(thresh, cv::Point(tmp_pt.x, tmp_pt.y), 5, cv::Scalar(0, 0, 255), -1);
    }
}

void drawObjects(cv::Mat& stats, cv::Mat& centroids, cv::Point& tmp_pt, cv::Mat& thresh) {
    for (int i = 1; i < stats.rows; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));

            // 현재 추적 중인 객체는 빨간색, 나머지는 파란색
            if (x == tmp_pt.x && y == tmp_pt.y) { 
                cv::rectangle(thresh, cv::Rect(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3)), cv::Scalar(0, 0, 255));
                cv::circle(thresh, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
            }
            else {
                cv::rectangle(thresh, cv::Rect(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3)), cv::Scalar(255, 0, 0));
                cv::circle(thresh, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
            }
        }
    }
}
 
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
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
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
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