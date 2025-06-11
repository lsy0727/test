#ifndef _VISION_HPP_
#define _VISION_HPP_

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void lidar_preprocess(Mat& frame, Mat& gray, Mat& thresh, std::vector<cv::Mat>& p_frame);
void findObjects(std::vector<cv::Mat>& p_frame, std::vector<cv::Mat>& stats, char& c); // preprocess_frame은 전처리 완료한 lframe, rframe을 넣음
void drawObjects(Mat& frame, std::vector<cv::Mat>& p_frame, std::vector<cv::Mat>& stats, std::vector<cv::Mat>& centroids, int& min_index);
int getError(Mat& thresh, Point& tmp_pt);


// void lidar_preprocess(Mat& frame, Mat& gray, Mat& thresh);
// void findObjects(Mat& thresh, Mat& stats, Mat& centroids, std::vector<std::vector<int>>& index); // preprocess_frame은 전처리 완료한 lframe, rframe을 넣음
// void drawObjects(Mat& frame, Mat& stats, Mat& centroids, std::vector<std::vector<int>>& index);
// int getError(Mat& thresh, Point& tmp_pt);

#endif //_VISION_HPP_