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
    modInput = input.clone();
    return;
}

MAP::~MAP()
{
    return;
}

int MAP::getCell(int u, int v) {
    int val =  (int)input.at<uchar>(v, u);
    return val;
}

void MAP::updateMap(int u, int v) {
    cv::circle(input, cv::Point(v, u), 10, cv::Scalar(0,0,0), cv::FILLED, 8,0);
    //cv::imshow("map", input);
    //cv::waitKey(0);
}

void MAP::updateMap(int u1, int v1, int u2, int v2) {
    cv::Point p1(u1, v1), p2(u2, v2);
    cv::Scalar colorLine(0, 255, 0); // Green
    cv::line(modInput, p1, p2, colorLine, 2);
    cv::circle(modInput, p1, 10, cv::Scalar(0,0,0), cv::FILLED, 2, 0);
    cv::circle(modInput, p2, 10, cv::Scalar(0,0,0), cv::FILLED, 2, 0);

    cv::imshow("map", modInput);
    cv::waitKey(0);
}
