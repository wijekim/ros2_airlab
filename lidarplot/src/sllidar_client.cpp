
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "opencv2/opencv.hpp"

#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define WIDTH 500
#define HEIGHT 500
#define MAXRANGE 2
#define SCALE ((WIDTH/2.0)/MAXRANGE)

cv::Mat img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
cv::VideoWriter video_writer;

void pointLidar(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

   
    int my_centerW = WIDTH / 2;
    int my_centerH = HEIGHT / 2;
    
    
    cv::circle(img, cv::Point(my_centerW, my_centerH), 3, cv::Scalar(255, 0, 0), -1);


    int count = scan->scan_time / scan->time_increment;
    for (int i = 0; i < count; i++) {
        float r = scan->ranges[i];

        float angle = scan->angle_min + scan->angle_increment * i;
        
        // 좌표 변환 
        float x = r * sin(angle);
        float y = r * cos(angle);

        // 이미지 좌표계로 변환
        //가로
        int px = my_centerW + (int)(x * SCALE);
        //세로 -를 +주게 되면 상하 반전
        int py = my_centerH + (int)(y * SCALE); 

        // 범위 체크 및 그리기
        if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
           cv::circle(img, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
        }
    }
}

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int count = scan->scan_time / scan->time_increment;
    printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

    img = cv::Scalar(255, 255, 255);
    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
        pointLidar(scan);

    }
    cv::imshow("Lidar_point",img);
    if (video_writer.isOpened()) {
        video_writer.write(img);
    }
    
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sllidar_client");

    std::string filename = "lidar_output.mp4";
    int fcc = cv::VideoWriter::fourcc('m','p','4','v'); // 코덱 설정
    double fps = 10.0; // 예상되는 LIDAR 데이터 수신 주기
    video_writer.open(filename, fcc, fps, cv::Size(WIDTH, HEIGHT));

    if (!video_writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "비디오 파일을 열 수 없습니다!");
    } else {
        RCLCPP_INFO(node->get_logger(), "비디오 녹화 시작: %s", filename.c_str());
    }
   
    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                            "scan", rclcpp::SensorDataQoS(), scanCb);

    rclcpp::spin(node);

    rclcpp::shutdown();


    return 0;
}
