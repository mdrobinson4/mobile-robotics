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
    threshold(input_gray, input_image, 150, 255, cv::THRESH_BINARY);
    modInput = input_image.clone();
    return;
}

bool MAP::isCollision(Eigen::Vector3d q) {
    int val = getCell(q(0), q(1));

    if (val == 255) // no collision = white
        return false;
    else if (val == 0) // collision = black
        return true;
}

bool MAP::isCollision(Eigen::Vector3d pnt0, Eigen::Vector3d pnt1) {
    int x1 = pnt0(0); int y1 = pnt0(1); float theta1 = pnt0(2);
    int x2 = pnt1(0); int y2 = pnt1(1); float theta2 = pnt1(2);

    cv::LineIterator it(input_image, cv::Point(x1, y1), cv::Point(x2, y2), 8);
    for(int i = 0; i < it.count; i++, ++it) {
        cv::Point pt = it.pos();
        float dot = x1*x2 + y1*y2; //      # dot product between [x1, y1] and [x2, y2]
        float det = x1*y2 - y1*x2;  //    # determinant
        float theta = atan2(det, dot);
        if (isCollision(Eigen::Vector3d(pt.x, pt.y, theta)) == true) {
          return true;
      } 
    }
    return false;
}

MAP::~MAP() {
    return;
}

int MAP::getCell(int u, int v) {
    int val =  (int)input_image.at<uchar>(v, u);
    return val;
}

void MAP::updateMap(Eigen::Vector3d pnt1, Eigen::Vector3d pnt2) {
    int u1 = pnt1(0); int v1 = pnt1(1);
    int u2 = pnt2(0); int v2 = pnt2(1);
    cv::Point p1(u1, v1), p2(u2, v2);
    cv::Scalar colorLine(0, 255, 0); // Green
    cv::line(modInput, p1, p2, colorLine, 2);
    cv::circle(modInput, p1, 5, cv::Scalar(0,0,0), cv::FILLED, 2, 0);
    cv::circle(modInput, p2, 5, cv::Scalar(0,0,0), cv::FILLED, 2, 0);
    cv::imshow("map", modInput);
    cv::waitKey(1);
}


void MAP::updateMap(Eigen::Vector3d pt) {
    int u = pt(0);
    int v = pt(1);
    cv::Point pnt(u, v);
    cv::Scalar color(0, 255, 0);
    cv::drawMarker(modInput, pnt, color, cv::MARKER_CROSS, 20, 10);
    cv::imshow("map", modInput);
    //cv::waitKey(0);
}

void MAP::clear() {
    modInput = input_image.clone();
}
