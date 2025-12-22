#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <chrono>
#include <math.h>

#define WIDTH 500
#define HEIGHT 500
#define MAXRANGE 2.0
#define SCALE ((WIDTH/2.0)/MAXRANGE)
#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std::chrono;

cv::Mat display_img; 
cv::VideoWriter video_writer;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dxl_pub;

// 전역 변수 (시각화를 위해 processImageToAlgorithm에서 계산된 값 공유)
struct DetectionResult {
    int error;
    float left_dist, left_angle;
    float right_dist, right_angle;
    bool left_found, right_found;
};

DetectionResult processImageToAlgorithm(cv::Mat frame) {
    int center_x = WIDTH / 2;
    int center_y = HEIGHT / 2;
    
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_red1(0, 100, 100), upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100), upper_red2(179, 255, 255);
    cv::Mat mask1, mask2;
    cv::inRange(hsv, lower_red1, upper_red1, mask1);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);
    mask = mask1 | mask2;

    DetectionResult res;
    res.left_dist = 1.0; res.right_dist = 1.0;
    res.left_found = false; res.right_found = false;

    for (int y = 0; y < mask.rows; y++) {
        for (int x = 0; x < mask.cols; x++) {
            if (mask.at<uchar>(y, x) > 0) {
                float dx = (x - center_x) / SCALE;
                float dy = (center_y - y) / SCALE;
                float r = sqrt(dx*dx + dy*dy);
                float angle = atan2(dx, dy);

                if (angle > M_PI/2.0 && angle <= M_PI) { // 좌측
                    if (r < res.left_dist) {
                        res.left_dist = r; res.left_angle = angle; res.left_found = true;
                    }
                } else if (angle >= -M_PI && angle < -M_PI/2.0) { // 우측
                    if (r < res.right_dist) {
                        res.right_dist = r; res.right_angle = angle; res.right_found = true;
                    }
                }
            }
        }
    }

    float target_angle = 0.0;
    const float MAX_VIEW = M_PI / 2.0;
    if (res.left_found && res.right_found) target_angle = (res.left_angle + res.right_angle) / 2.0;
    else if (res.left_found) target_angle = (res.left_angle + (-MAX_VIEW)) / 2.0;
    else if (res.right_found) target_angle = (res.right_angle + MAX_VIEW) / 2.0;

    res.error = (int)RAD2DEG(target_angle);
    return res;
}

void imageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;

    cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));
    display_img = frame.clone();

    // 1. 알고리즘 실행 및 데이터 획득
    DetectionResult res = processImageToAlgorithm(frame);
    
    // 2. 제어값 발행
    float k = 0.5;
    int speed = 50;
    geometry_msgs::msg::Vector3 vel;
    vel.x = (float)(speed - k * res.error);
    vel.y = (float)-(speed + k * res.error);
    dxl_pub->publish(vel);

    // 3. 시각화 로직 추가 (사용자 요청 부분)
    int center_px = WIDTH / 2;
    int center_py = HEIGHT / 2;
    int rect_size = 10;

    // 좌측 장애물 표시
    if(res.left_found) {
        int px = center_px + (int)(res.left_dist * sin(res.left_angle) * SCALE);
        int py = center_py - (int)(res.left_dist * cos(res.left_angle) * SCALE);

        int edge_x = (px < center_px) ? px + rect_size : px - rect_size;
        int edge_y = (py < center_py) ? py + rect_size : py - rect_size;

        cv::line(display_img, cv::Point(center_px, center_py), cv::Point(edge_x, edge_y), cv::Scalar(0, 255, 0), 1);
        cv::rectangle(display_img, cv::Point(px - rect_size, py - rect_size), cv::Point(px + rect_size, py + rect_size), cv::Scalar(255, 0, 0), 2);
        cv::circle(display_img, cv::Point(px, py), 2, cv::Scalar(255, 0, 0), -1);
    }

    // 우측 장애물 표시
    if(res.right_found) {
        int px = center_px + (int)(res.right_dist * sin(res.right_angle) * SCALE);
        int py = center_py - (int)(res.right_dist * cos(res.right_angle) * SCALE);

        int edge_x = (px < center_px) ? px + rect_size : px - rect_size;
        int edge_y = (py < center_py) ? py + rect_size : py - rect_size;

        cv::line(display_img, cv::Point(center_px, center_py), cv::Point(edge_x, edge_y), cv::Scalar(0, 0, 255), 1);
        cv::rectangle(display_img, cv::Point(px - rect_size, py - rect_size), cv::Point(px + rect_size, py + rect_size), cv::Scalar(255, 0, 0), 2);
        cv::circle(display_img, cv::Point(px, py), 2, cv::Scalar(255, 0, 0), -1);
    }

    // 조향 방향 표시 (하늘색 선)
    float line_angle = (float)res.error * M_PI / 180.0;
    cv::line(display_img, cv::Point(center_px, center_py), 
             cv::Point(center_px + 50*sin(line_angle), center_py - 50*cos(line_angle)), 
             cv::Scalar(255, 255, 0), 3);

    cv::imshow("Simulation_Visual", display_img);
    if (video_writer.isOpened()) video_writer.write(display_img);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("video_algo_visual_node");
    dxl_pub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", 10);
    auto image_sub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", rclcpp::QoS(10), imageCb);
    video_writer.open("sim_result_visual.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 10.0, cv::Size(WIDTH, HEIGHT));
    rclcpp::spin(node);
    video_writer.release();
    rclcpp::shutdown();
    return 0;
}