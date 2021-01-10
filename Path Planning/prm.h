#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <utility>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "map.h"

using namespace std;

struct Node {
    int x;
    int y;
};

class PRM
{
private:
    int n; // number of nodes to put in roadmap
    int k; // number of closest neighbors to examine for each configuration
    Eigen::Vector3d *V;
    int width;
    int height;
    int vector_cnt;
    int init_idx;
    int goal_idx;
    cv::Mat input_image;
    MAP *map;
    vector<pair<int, float>> *graph;
public:
    PRM(string, int, int);
    PRM(cv::Mat, int, int);
    void clearRoadmap();
    void drawMap();
    void setCoords(Eigen::Vector3d q_init, Eigen::Vector3d q_goal);
    int addNode(Eigen::Vector3d q_new);
    void removeNode(int);
    void addEdge(int u_idx, int v_idx, float weight);
    void connectNodes(Eigen::Vector3d q_init, Eigen::Vector3d q_goal);
    vector<pair<int, float>> nearestNeighbors(Eigen::Vector3d q);
    bool checkDuplicateEdge(int u, int v);
    bool checkDuplicateEdge(Eigen::Vector3d);
    bool bresenham(int x1, int y1, int x2, int y2);
    void constructRoadmap();
    ~PRM();
};