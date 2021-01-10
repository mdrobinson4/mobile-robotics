#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "prm.h"

using namespace std;

int main() {
    int n = 100;
    int k = 10;

    cv::Mat input_image = cv::imread("map3.jpg");
    Eigen::Vector3d q_init(203, 430, 0);
    Eigen::Vector3d q_goal(1306, 69, 0);

    PRM *prm = new PRM(input_image, n, k);
    prm->constructRoadmap();

    prm->setCoords(q_init, q_goal);
    prm->drawMap();
    prm->clearRoadmap();
    prm->setCoords(Eigen::Vector3d(42, 531, 0), Eigen::Vector3d(1310, 752, 0));
    prm->drawMap();
    return 0;
}
