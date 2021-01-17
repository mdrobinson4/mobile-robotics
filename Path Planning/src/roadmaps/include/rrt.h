#ifndef RRT_H
#define RRT_H

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
#include "astar.h"
#include "bfs.h"
#include "dfs.h"

using namespace std;

typedef priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> MIN_QUEUE;

class RRT {
private:
    class Dijkstra *dijkstra;
    class ASTAR *astar;
    class BFS *bfs;
    class DFS *dfs;
    int n; // number of nodes to put in roadmap
    int k; // number of closest neighbors to examine for each configuration
    MAP *map;
    int width;
    int height;
    int node_cnt;
    double step_size;
    bool found_path;
    int init_node;
    int goal_node;
    cv::Mat input_image;
    Eigen::Vector3d *configs;
    vector<pair<int, double>> *graph;
public:
    RRT(cv::Mat, double, int, int);
    RRT(string, int, int);
    ~RRT();
    void search();
    void buildRRT(Eigen::Vector3d, Eigen::Vector3d);
    void drawMap();
    void clearCoords();
private:
    bool extend(Eigen::Vector3d);
    Eigen::Vector3d randConfig();
    Eigen::Vector3d newConfig(Eigen::Vector3d, Eigen::Vector3d);
    int nearestNeighbor(Eigen::Vector3d);
    double computeDistance(Eigen::Vector3d, Eigen::Vector3d);
    void addNode(Eigen::Vector3d);
    void addEdge(int, int, double);
    int checkDuplicateEdge(Eigen::Vector3d);
};

#endif