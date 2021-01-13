#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "prm.h"
//#include "rrt.h"

using namespace std;

int main() {
    int n = 1000;
    int k = 10;

    string path("../map3.jpg");
    cv::Mat input_image = cv::imread(path);
    Eigen::Vector3d q_init(203, 430, 0);
    Eigen::Vector3d q_goal(1306, 69, 0);

    PRM *prm = new PRM(input_image, n, k);
    prm->constructRoadmap();
    prm->setCoords(q_init, q_goal);
    prm->drawMap();
    prm->search();
    
    prm->clearCoords();
    prm->setCoords(Eigen::Vector3d(203, 430, 0), Eigen::Vector3d(310, 42, 0));
    prm->drawMap();
    prm->search();
    return 0;
}
