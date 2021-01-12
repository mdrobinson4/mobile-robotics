#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;

class MAP
{
private:
    cv::Mat input_image;
    cv::Mat modInput;
public:
    MAP(cv::Mat input);
    void clear();
    int getCell(int u, int v);
    void updateMap(Eigen::Vector3d, Eigen::Vector3d);
    void updateMap(Eigen::Vector3d);
    bool isCollision(Eigen::Vector3d);
    bool isCollision(Eigen::Vector3d, Eigen::Vector3d);
    ~MAP();
};