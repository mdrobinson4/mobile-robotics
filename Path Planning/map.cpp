#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "map.h"

using namespace std;

MAP::MAP(cv::Mat input_)
{
    cv::Mat input_gray;
    cv::cvtColor(input_, input_gray, cv::COLOR_BGR2GRAY);
    threshold(input_gray, input, 150, 255, cv::THRESH_BINARY);
    return;
}

MAP::~MAP()
{
    return;
}

int MAP::getCell(int u, int v) {
    return (int)input.at<uchar>(u, v);
}

void MAP::updateMap(int u, int v) {
    cv::circle(input, cv::Point(v, u), 10, cv::Scalar(0,0,0), cv::FILLED, 8,0);
    cv::imshow("map", input);
    cv::waitKey(0);
}
