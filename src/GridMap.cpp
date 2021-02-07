#include "GridMap.h"

GridMap::GridMap() {}

void GridMap::GridMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cld, Eigen::Matrix4d pose) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cld = pclToGrid(new_cld);
    if (2dmap->empty()) {
        xx = pclToGrid(new_cloud);
    }
}
    //GridMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, cv::Mat, Eigen::Matrix4d);
private:
    RRT *rrt;
    PRM *prm;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
}