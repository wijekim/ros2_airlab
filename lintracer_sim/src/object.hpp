#ifndef _OBJECT_HPP_
#define _OBJECT_HPP_

#include <iostream>
#include "opencv2/opencv.hpp"


using namespace cv;
using namespace std;

void findObjects(Mat& binary, Point& tmp_pt, Mat& stats, Mat& centroids);
void drawObjects(Mat& stats, Mat& centroids, Point& tmp_pt, Mat& binary);
int Errorobject(cv::Mat& binary, cv::Point& tmp_pt);
#endif 