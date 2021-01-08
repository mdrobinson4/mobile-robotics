#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <math.h>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "prm.h"

using namespace std;

PRM::PRM(string path, int n_, int k_)
{
    n = n_;
    k = k_;
    cv::Mat image = cv::imread(path);
    width = image.cols - 1; // x
    height = image.rows - 1; // y
    map = new MAP(image);

    V = new Eigen::Vector2d[n];
    graph = new vector<pair<int, float>>[n];
    return;
}

void PRM::roadmapConstruction() {
    int vector_cnt = 0;
    while (vector_cnt < n) {
        int x = rand() % width;
        int y = rand() % height;
        Eigen::Vector2d pos(x, y);
        bool collision = checkCollision(pos);
        bool duplicate = false;
        
        if (collision == true) {
            for (int i = 0; i < vector_cnt; i++) {
                if (pos.isApprox(V[i])) {
                    duplicate = true;
                    break;
                }
            }
        }
        if (collision == false && duplicate == false) {
            V[vector_cnt] = pos;
            std::pair <int, float> q(vector_cnt, -1);
            graph[vector_cnt].push_back(q);
            vector_cnt += 1;
            //map->updateMap(pos(1), pos(0));
        }
    }
    for (int i = 0; i < vector_cnt; i++) {
        Eigen::Vector2d q = V[i];
        vector<Eigen::Vector2d> q_nearest = nearestNeighbors(q);
    }
}

bool PRM::checkCollision(Eigen::Vector2d q) {
    bool res;
    int val = map->getCell(q(1), q(0));

    if (val == 255) // no collision = black
        res = false;
    else if (val == 0) // collision = black
        res = true;

    return res;
}

typedef pair<float, vector<int>> pi;

vector<Eigen::Vector2d> PRM::nearestNeighbors(Eigen::Vector2d q) {
    priority_queue<pi, vector<pi>, greater<pi>> heap;
    for (int i = 0; i < n; i++) {
        if (q != V[i]) {
        float dx = abs(q(0) - V[i](0));
        float dy = abs(q(1) - V[i](1));
        double d = sqrt(pow(dx, 2) + pow(dy, 2));
        //cout << d << endl;
        vector<int> arr = {int(V[i](0)), int(V[i](1))};
        heap.push(make_pair(d, arr));
        }
    }
    cout << "(" << q(0) << ", " << q(1) << ")" << endl << "++++++++++++++" << endl;
    vector<Eigen::Vector2d> neighbors(k);
    for (int i = 0; i < k; i++) {
        pair<float, vector<int>> h = heap.top();
        cout << "(" << h.second[0] << ", " << h.second[1] << ")" << " = " << h.first << endl;
        neighbors.push_back(Eigen::Vector2d(h.second[0], h.second[1]));
        heap.pop();
    }
    cout << "____________" << endl;
    return neighbors;
}

PRM::~PRM()
{
    return;
}