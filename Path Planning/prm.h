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

#include "map.h"
#include "dijkstra.h"

using namespace std;

typedef priority_queue<pair<float, int>, vector<pair<float, int>>, greater<pair<float, int>>> MIN_QUEUE;

class PRM {
private:
    class Dijkstra *dijkstra;
    int n; // number of nodes to put in roadmap
    int k; // number of closest neighbors to examine for each configuration
    MAP *map;
    int width;
    int height;
    int init_idx;
    int goal_idx;
    int vector_cnt;
    Eigen::Vector3d *V;
    cv::Mat input_image;
    vector<pair<int, float>> *graph;
public:
    PRM(string, int, int);
    PRM(cv::Mat, int, int);
    ~PRM();
    void search();
    void clearCoords();
    void drawMap();
    void setCoords(Eigen::Vector3d, Eigen::Vector3d);
    void constructRoadmap();
private:
    int addNode(Eigen::Vector3d);
    void removeNode(int);
    void addEdge(int u_idx, int v_idx, float weight);
    void connectNodes(Eigen::Vector3d q_init, Eigen::Vector3d q_goal);
    MIN_QUEUE nearestNeighbors(Eigen::Vector3d q);
    bool checkDuplicateEdge(int u, int v);
    int checkDuplicateEdge(Eigen::Vector3d);
    bool bresenham(int x1, int y1, int x2, int y2);
};