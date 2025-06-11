#include "/home/linux/ros2_ws/src/lidardrive/include/lidardrive/vision.hpp"
#include <vector>
using namespace std;
using namespace cv;

void lidar_preprocess(Mat& frame, Mat& gray, Mat& thresh, std::vector<cv::Mat>& p_frame) {
    //전처리
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    Scalar bright_avg = mean(gray); // 밝기 평균
    gray = gray + (100 - bright_avg[0]);
    threshold(gray, thresh, 100, 255, THRESH_BINARY);

    //자르기
    Rect left_r(0, 0, thresh.cols / 2, thresh.rows / 2);
    Rect right_r(thresh.cols / 2, 0, thresh.cols / 2, thresh.rows / 2);
    p_frame[0] = thresh(left_r);
    p_frame[1] = thresh(right_r);
}


// 객체 찾기
void findObjects(std::vector<cv::Mat>& p_frame, std::vector<cv::Mat>& stats, std::vector<int>& index, std::vector<cv::Point>& point) {
    Mat labels, centroids;
    int x, y;
    for (int n = 0; n = p_frame.size(); n++) {
        if (n == 0) // 왼쪽 영역
        {
            // 객체 검출
            int cnt = connectedComponentsWithStats(p_frame[0], labels, stats[0], centroids);

            // 1채널 -> 3채널 변환
            cvtColor(p_frame[n], p_frame[n], COLOR_GRAY2BGR);
            
            int l_x, l_y, l_index, l_min_dist = 500, dist;
            for (int i = 1; i < cnt; i++)
            {
                // 객체 바운딩박스의 우측 하단 x, y좌표
                x = stats[n].at<int>(i, 0) + stats[n].at<int>(i, 2);
                y = stats.[n].at<int>(i, 1) + stats[n].at<int>(i, 3);

                dist = norm(Point(x, y) - Point(250, 250));
                if (dist < l_min_dist)
                {
                    l_min_dist = dist;
                    l_x = x;
                    l_y = y;
                }
            }
            point[n] = Point(l_x, l_y);
            index.push_back(l_index);
        }
        else if (n == 1)    // 오른쪽 영역
        {
            // 객체 검출
            int cnt = connectedComponentsWithStats(p_frame[n], labels, stats[n], centroids);

            // 1채널 -> 3채널 변환
            cvtColor(p_frame[n], p_frame[n], COLOR_GRAY2BGR);

            int r_x, r_y, r_index, r_min_dist = 500, dist;
            for (int i = 1; i < cnt; i++)
            {
                // 객체 바운딩박스의 좌측 하단 x,y 좌표
                x = stats[n].at<int>(i, 0);
                y = stats[n].at<int>(i, 1) + stats[n].at<int>(i, 3);

                dist = norm(Point(x, y) - Point(0, 250));
                if (dist < r_min_dist)
                {
                    r_min_dst = dist;
                    r_x = x;
                    r_y = y;

                }
            }
            point[n] = Point(r_x, r_y);
            index.push_back(r_index);
        }
    }
}

// 객체 표시
void drawObjects(Mat& frame, std::vector<cv::Mat>& stats, std::vector<int>& index, std::vector<cv::Point>& point) {
    int top;
    rectangle(frame, Rect(stats.at<int>(min_index, 0), stats.at<int>(min_index, 1), stats.at<int>(min_index, 2), stats.at<int>(min_index, 3)), Scalar(255, 0, 0));
    line(frame, Point(250, 250), Point(x, y), Scalar(0, 255, 0), 2, LINE_4, 0);
}

// error 계산
int getError(Mat& thresh, Point& tmp_pt) {
    return ((thresh.cols / 2) - tmp_pt.x);
}




// void lidar_preprocess(Mat& frame, Mat& gray, Mat& thresh) {
//     //전처리
//     cvtColor(frame, gray, COLOR_BGR2GRAY);
//     Scalar bright_avg = mean(gray); // 밝기 평균
//     gray = gray + (100 - bright_avg[0]);
//     threshold(gray, thresh, 50, 255, THRESH_BINARY);
//     thresh = ~thresh;

//     //자르기
//     Rect r(0, 0, thresh.cols, thresh.rows / 2);
//     thresh = thresh(r);
// }

// // 완전히 포함되어 있는 경우 (내부에 완전히 들어갔을 때 true)
// bool isInside(const cv::Rect& r) {
//     return r.area() > 0;
// }

// // 경계가 겹치기만 해도 true
// bool isTouching(const cv::Rect& r) {
//     return r.area() > 0;
// }


// // 객체 찾기
// void findObjects(Mat& thresh, Mat& stats, Mat& centroids, std::vector<std::vector<int>>& index) {
//     // 객체검출
//     Mat labels;
//     int cnt = connectedComponentsWithStats(thresh, labels, stats, centroids);

//     // 1채널 -> 3채널 변환
//     cvtColor(thresh, thresh, COLOR_GRAY2BGR);

//     // 검출할 영역을 y축을 기준으로 절반으로 나눔
//     Rect left_r(0, 0, thresh.cols / 2, thresh.rows / 2);
//     Rect right_r(thresh.cols / 2, 0, thresh.cols / 2, thresh.rows / 2);

//     for (int i = 1; i < cnt; i++) {
//         Rect obj(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3) // 객체 바운딩박스 범위
//         );
//         // isInside(a & b) : b가 a안에 있으면 true반환
//         if (isInside(left_r & obj)) {   // 객체가 왼쪽 영역에 있는 경우
//             index[0].push_back(i);
//         }
//         else if (isInside(right_r & obj)) { // 객체가 오른쪽 영역에 있는 경우
//             index[1].push_back(i);
//         }
//         // isTouching(a & b) : b가 a에 조금이라도 포함되어있으으면 true반환
//         else if (isTouching(left_r & obj) || isTouching(right_r & obj)) {   // 객체가 중앙에 걸쳐있는 경우
//             index[2].push_back(i);
//         }

//         // int x = stats.at<int>(i, 0);    // 객체 바운딩박스의 좌측 하단 x좌표
//         // int y = stats.at<int>(i, 1) + stats.at<int>(i, 3);  // 객체 바운딩박스의 좌측 하단 y좌표
//         // int dist = norm(Point(x, y) - Point(250, 250));   //거리 계산
        
//         // if (dist < min_dist) {   //거리가 이전 객체보다 짧은 경우
//         //     min_dist = dist;
//         //     min_index = i;
//         // }
//     }
// }

// // 객체 표시
// void drawObjects(Mat& frame, Mat& stats, Mat& centroids, std::vector<std::vector<int>>& index) {
//     int l_x, l_y, l_min_index = -1, l_min_dist = 500;
//     for (int objNum : index[0]) {   // 왼쪽 영역
//         l_x = stats.at<int>(objNum, 0) + stats.at<int>(objNum, 2);    // 객체 바운딩박스의 우측 하단 x좌표
//         l_y = stats.at<int>(objNum, 1) + stats.at<int>(objNum, 3);  // 객체 바운딩박스의 우측 하단 y좌표

//         int dist = norm(Point(l_x, l_y) - Point(250, 250));
//         if (dist < l_min_dist) {
//             l_min_dist = dist;
//             l_min_index = objNum;
//         }
//     }
//     int r_x, r_y, r_min_index = -1, r_min_dist = 500;
//     for (int objNum : index[1]) {   // 오른쪽 영역
//         r_x = stats.at<int>(objNum, 0); // 객체 바운딩박스의 좌측 하단 x좌표
//         r_y = stats.at<int>(objNum, 1) + stats.at<int>(objNum, 3); // 객체 바운딩박스의 좌측 하단 y좌표

//         int dist = norm(Point(r_x, r_y) - Point(250, 250));
//         if (dist < r_min_dist) {
//             r_min_dist = dist;
//             r_min_index = objNum;
//         }
//     }
//     int f_x, f_y, f_min_index = -1, f_min_dist = 500;
//     for (int objNum : index[2]) {   // 중앙 객체
//         f_x = stats.at<int>(objNum, 0) + stats.at<int>(objNum, 2) / 2;    // 객체 바운딩박스의 중앙 하단 x좌표
//         f_y = stats.at<int>(objNum, 1) + stats.at<int>(objNum, 3);  // 객체 바운딩박스의 중앙 하단 y좌표

//         int dist = norm(Point(f_x, f_y) - Point(250, 250));
//         if (dist < f_min_dist) {
//             f_min_dist = dist;
//             f_min_index = objNum;
//         }
//     }
//     rectangle(frame, Rect(stats.at<int>(l_min_index, 0), stats.at<int>(l_min_index, 1), stats.at<int>(l_min_index, 2), stats.at<int>(l_min_index, 3)), Scalar(255, 0, 0));
//     line(frame, Point(250, 250), Point(l_x, l_y), Scalar(255, 0, 0), 2, LINE_4, 0);

//     rectangle(frame, Rect(stats.at<int>(r_min_index, 0), stats.at<int>(r_min_index, 1), stats.at<int>(r_min_index, 2), stats.at<int>(r_min_index, 3)), Scalar(0, 255, 0));
//     line(frame, Point(250, 250), Point(r_x, r_y), Scalar(0, 255, 0), 2, LINE_4, 0);

//     rectangle(frame, Rect(stats.at<int>(f_min_index, 0), stats.at<int>(f_min_index, 1), stats.at<int>(f_min_index, 2), stats.at<int>(f_min_index, 3)), Scalar(255, 0, 0));
// }

// // error 계산
// int getError(Mat& thresh, Point& tmp_pt) {
//     return ((thresh.cols / 2) - tmp_pt.x);
// }