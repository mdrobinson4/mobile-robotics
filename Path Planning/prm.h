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
    Eigen::Vector2d *V;
    int width;
    int height;
    MAP *map;
    vector<pair<int, float>> *graph;
public:
    PRM(string, int, int);
    vector<pair<int, float>> nearestNeighbors(Eigen::Vector2d q);
    bool checkCollision(Eigen::Vector2d q);
    bool checkCollision(Eigen::Vector2d a, Eigen::Vector2d b);
    bool checkDuplicateEdge(int u, int v);
    void roadmapConstruction();
    ~PRM();
};