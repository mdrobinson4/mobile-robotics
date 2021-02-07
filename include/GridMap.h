#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <utility>
#include <stack>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "rrt.h"
#include "prm.h"

class GridMap {
public:
    GridMap();
    void insert(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, cv::Mat, Eigen::Matrix4d);
    //GridMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, cv::Mat, Eigen::Matrix4d);
private:
    RRT *rrt;
    PRM *prm;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
}

/*
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
*/