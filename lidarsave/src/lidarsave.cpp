
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <chrono>
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define WIDTH 500
#define HEIGHT 500
#define MAXRANGE 2
#define SCALE ((WIDTH/2.0)/MAXRANGE)

using namespace std::chrono;

cv::Mat img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
cv::VideoWriter video_writer;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dxl_pub;

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
        int py = my_centerH - (int)(y * SCALE); 

        // 범위 체크 및 그리기
        if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
           cv::circle(img, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
        }
    }
}

int objectFind(sensor_msgs::msg::LaserScan::SharedPtr scan){
    //반경
    float left_dist = 1.0; 
    float right_dist = 1.0;

    float left_angle = 0.0;
    float right_angle = 0.0;

    bool left_found = false;
    bool right_found = false;

    int center_px = WIDTH / 2;
    int center_py = HEIGHT / 2;
    
    //전체 데이터 수
    int count = scan->scan_time / scan->time_increment;

    for(int i = 0; i < count; i++){
        float r = scan->ranges[i];
        float angle = scan->angle_min + scan->angle_increment * i;

        //좌측 전방 (M_PI/2.0) <- 90 / -90 ~ 0
        // if(angle>0 && angle <= M_PI/2.0)
        if(angle>M_PI/2.0 && angle <= M_PI){
            //기존에 찾은 장애물보다 가까울때
            if(r < left_dist){
                left_dist = r;
                left_angle = angle;
                left_found = true;
            }
        }
        else if(angle>= -M_PI && angle < -M_PI/2.0){
            if(r < right_dist){
                right_dist = r;
                right_angle = angle;
                right_found = true;
            }
        }
    }
    float target_angle = 0.0;
    const float MAX_VIEW = M_PI / 2.0; // 90도 /이미지의 전방 180도

    if (left_found && right_found) {
        // 양쪽 다 있을 때: 두 장애물 사이의 정중앙
        target_angle = (left_angle + right_angle) / 2.0;
    } 
    else if (left_found && !right_found) {
        // 좌측에만 있을 때: 좌측 장애물과 우측 끝(-90도) 사이의 중앙
        // 장애물이 +45도에 있다면, (45 + (-90)) / 2 = -22.5도 조준
            target_angle = (left_angle + (-MAX_VIEW)) / 2.0;
    } 
    else if (!left_found && right_found) {
        // 우측에만 있을 때: 우측 장애물과 좌측 끝(+90도) 사이의 중앙
        // 장애물이 -30도에 있다면, (-30 + 90) / 2 = +30도 조준
        target_angle = (right_angle + MAX_VIEW) / 2.0;
    } 
    else {
        //장애물 없음 정면 유지
        target_angle = 0.0;
    }

    int rect_size = 10;
    if(left_found) {
        int px = (WIDTH/2) + (int)(left_dist * sin(left_angle) * SCALE);
        int py = (HEIGHT/2) - (int)(left_dist * cos(left_angle) * SCALE);

        // 로봇 중심(center_px, center_py)과 비교하여 사각형의 4개 꼭짓점 중 하나 선택
        int edge_x = (px < center_px) ? px + rect_size : px - rect_size;
        int edge_y = (py < center_py) ? py + rect_size : py - rect_size;


        cv::Point closest_edge(edge_x, edge_y);
        //cv::line(img, cv::Point(center_px, center_py), cv::Point(px, py), cv::Scalar(0,0 ,255), 1);
        cv::line(img, cv::Point(center_px, center_py), closest_edge, cv::Scalar(0, 255, 0), 1);

        //cv::circle(img, cv::Point(px, py), 8, cv::Scalar(0, 255, 255), 2); // 노란색 테두리
        cv::rectangle(img, 
                  cv::Point(px - rect_size, py - rect_size), 
                  cv::Point(px + rect_size, py + rect_size), 
                  cv::Scalar(255, 0, 0), 2);
        cv::circle(img, cv::Point(px, py), 2, cv::Scalar(255, 0, 0), -1); // 노란색 점
    }
    // 우측 장애물 표시 (노란색 원)
    if(right_found) {
        int px = (WIDTH/2) + (int)(right_dist * sin(right_angle) * SCALE);
        int py = (HEIGHT/2) - (int)(right_dist * cos(right_angle) * SCALE);

        int edge_x = (px < center_px) ? px + rect_size : px - rect_size;
        int edge_y = (py < center_py) ? py + rect_size : py - rect_size;

        cv::Point closest_edge(edge_x, edge_y);

        
        cv::line(img, cv::Point(center_px, center_py), closest_edge, cv::Scalar(0, 0, 255), 1);

        //cv::circle(img, cv::Point(px, py), 8, cv::Scalar(0, 255, 255), 2);
        cv::rectangle(img, 
                  cv::Point(px - rect_size, py - rect_size), 
                  cv::Point(px + rect_size, py + rect_size), 
                  cv::Scalar(255, 0, 0), 2);
        cv::circle(img, cv::Point(px, py), 2, cv::Scalar(255, 0, 0), -1);
    }

    return (int)RAD2DEG(target_angle);

}
int errorControl(sensor_msgs::msg::LaserScan::SharedPtr scan, float &return_Lvel, float &return_Rvel){
    double k =0.5;
    int speed =50;
    int error = objectFind(scan);

    geometry_msgs::msg::Vector3 vel;
    vel.x = (float)(speed - k * error);  // Left Motor
    vel.y = (float)-(speed + k * error); // Right Motor 

    dxl_pub->publish(vel);

    return_Lvel = vel.x;
    return_Rvel = vel.y;

    return error;
}


static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {

    auto start_time = steady_clock::now();
    img = cv::Scalar(255, 255, 255); // 배경 초기화

    // 1. 기본 LiDAR 점 그리기 (빨간색)
    pointLidar(scan);

    // 2. 장애물 찾기 및 제어값 계산
    float return_Lvel = 0, return_Rvel = 0;
    int error = errorControl(scan, return_Lvel, return_Rvel);
   

   
    float line_angle = (float)error * M_PI / 180.0;
    
    cv::flip(img, img, 1);
    cv::flip(img, img, -1); // 상하반전 (로봇 방향에 맞춰)
    
    // 조향 방향 표시 (파란색 선)
    cv::line(img, cv::Point(WIDTH/2, HEIGHT/2), 
             cv::Point(WIDTH/2 + 50*sin(line_angle), HEIGHT/2 - 50*cos(line_angle)), 
             cv::Scalar(255, 0, 0), 2);

    
    auto end_time = steady_clock::now();
    float totalTime = duration<float, std::milli>(end_time - start_time).count();

    printf("error: %d / Time:%.2f\n",error,totalTime);
    printf("Lvel:%.2f / Rvel:%.2f\n",return_Lvel,return_Rvel);
    cv::imshow("Lidar_point",img);
    if (video_writer.isOpened()) {
        video_writer.write(img);
    }
    
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sllidar_client");

    dxl_pub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", 10);

    std::string filename = "lidar_output.mp4";
    int fcc = cv::VideoWriter::fourcc('m','p','4','v'); // 코덱 설정
    double fps = 10.0; 
    video_writer.open(filename, fcc, fps, cv::Size(WIDTH, HEIGHT));

    if (!video_writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "비디오 파일을 열 수 없습니다!");
    } else {
        RCLCPP_INFO(node->get_logger(), "비디오 녹화 시작: %s", filename.c_str());
    }
   
    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), scanCb);

    
    rclcpp::spin(node);

    if (video_writer.isOpened()) {
        RCLCPP_INFO(node->get_logger(), "영상 저장을 완료하고 종료합니다.");
        video_writer.release();
    }

    rclcpp::shutdown();


    return 0;
}

