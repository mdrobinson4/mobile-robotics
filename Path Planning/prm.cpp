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

PRM::PRM(cv::Mat input_image, int n_, int k_) {
    init_idx = -1;
    goal_idx = -1;
    n = n_; // number of nodes to initialize
    k = k_; // number of neighbors
    vector_cnt = 0;
    
    width = input_image.cols - 1; // x
    height = input_image.rows - 1; // y
    map = new MAP(input_image);

    V = new Eigen::Vector3d[n + 2];
    graph = new vector<pair<int, float>>[n + 2];
    return;
}

void PRM::constructRoadmap() {
    int x;
    int y;
    float theta;
    vector_cnt = 0;
    while (vector_cnt < n) {
        x = rand() % width;
        y = rand() % height;
        theta = rand() % 360 - 1;
        Eigen::Vector3d pos(x, y, theta);
        
        if (!map->checkCollision(pos) && !checkDuplicateEdge(pos)) {
            V[vector_cnt] = pos;
            vector_cnt += 1;
        }
    }
    for (int i = 0; i < vector_cnt; i++) {
        int u_idx = i;
        int v_idx = 0;
        float weight;
        vector<pair<int, float>> qn = nearestNeighbors(V[i]);
        for (auto curr_neighbor : qn) {
            v_idx = curr_neighbor.first;
            weight = curr_neighbor.second;
            if (!checkDuplicateEdge(u_idx, v_idx) && !map->checkCollision(V[u_idx], V[v_idx])) {
                addEdge(u_idx, v_idx, weight);
            }
        }
    }
}

vector<pair<int, float>> PRM::nearestNeighbors(Eigen::Vector3d q) {
    float dx, dy, dist = 0;
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<pair<float, int>>> min_queue;
    for (int i = 0; i < n; i++) {
        if (q != V[i]) {
            dx = abs(q(0) - V[i](0));
            dy = abs(q(1) - V[i](1));
            dist = sqrt(pow(dx, 2) + pow(dy, 2));
            min_queue.push(make_pair(dist, i));
        }
    }
    vector<pair<int, float>> neighbor_idx;
    for (int i = 0; i < k; i++) {
        pair<float, int> node = min_queue.top();
        neighbor_idx.push_back({node.second, node.first});
        min_queue.pop();
    }
    return neighbor_idx;
}

void PRM::removeNode(int u) {
    int v = graph[u].back().first;
    if (u < n) { // start node already existed in graph
        graph[u].pop_back();
    }
    else {
        V[u] = Eigen::Vector3d();
        graph[u] = vector<pair<int, float>>();
        vector_cnt -= 1;
    }
    graph[v].pop_back();
}


void PRM::setCoords(Eigen::Vector3d q_init, Eigen::Vector3d q_goal) {
    if (init_idx >= 0) {
        int init_u_idx = init_idx; // index of newly added node (goal)
        int goal_u_idx = goal_idx; // index of newly added node (goal)
        removeNode(init_u_idx);
        removeNode(goal_u_idx);
    }

    init_idx = addNode(q_init);
    if (init_idx == -1) {
        cout << "element not added" << endl;
        return;
    }
    goal_idx = addNode(q_goal);
    if (goal_idx == -1) {
        removeNode(goal_idx);
    }
}

int PRM::addNode(Eigen::Vector3d q_new) {
    int u_idx; int v_idx;
    vector<pair<int, float>> qn_new = nearestNeighbors(q_new); // configurations near init
    // add q_init to vector list
    if (checkDuplicateEdge(q_new) == false) {
        Eigen::Vector3d q_curr;
        float dist;
        u_idx = vector_cnt;
        for (int i = 0; i < qn_new.size(); i++ ) {
            pair<int, float> curr_config = qn_new[i];
            v_idx = curr_config.first; // index of nearby configuration
            dist = curr_config.second;
            q_curr = V[v_idx];
            if (!map->checkCollision(q_new, q_curr)) {
                V[u_idx] = q_new;
                addEdge(u_idx, v_idx, dist);
                vector_cnt++;
                break;
            }
        }
    }
    else {
        for (int i = 0; i < vector_cnt; i++)
            if (q_new == (V[i]))
                return i;
    }
    return u_idx;
}

void PRM::addEdge(int u_idx, int v_idx, float weight) {
    graph[u_idx].push_back(make_pair(v_idx, weight));
    graph[v_idx].push_back(make_pair(u_idx, weight));
}

bool PRM::checkDuplicateEdge(Eigen::Vector3d pos) {
    for (int i = 0; i < vector_cnt; i++) {
        if (pos == (V[i])) {
            return true;
        }
    }
    return false;
}

bool PRM::checkDuplicateEdge(int u, int v) {
    const int X_v(u);
    const int X_u(v);

    if (graph[u].size() == 0 || graph[v].size() == 0) {
        return false;
    }

    auto it_u = std::find_if(graph[u].begin(), 
                        graph[u].end(), 
                        [&X_v](const pair<int, float>& p)
                        {return p.first == X_v; });

    auto it_v = std::find_if(graph[v].begin(), 
                        graph[v].end(), 
                        [&X_u](const pair<int, float>& p)
                        {return p.first == X_u; });

    if (it_u == graph[u].end() && it_v == graph[v].end()) {
        return true; // duplicate entry
    }
    else {
        return false; // new entry
    }
}

bool PRM::bresenham(int x1, int y1, int x2, int y2) {
   int m_new = 2 * (y2 - y1); 
   int slope_error_new = m_new - (x2 - x1); 
   for (int x = x1, y = y1; x <= x2; x++) {
      if (map->checkCollision(Eigen::Vector3d(x, y, 0)) == true) {
          return true;
      } 
      slope_error_new += m_new; 
      if (slope_error_new >= 0) { 
         y++; 
         slope_error_new  -= 2 * (x2 - x1); 
      } 
      if (map->checkCollision(Eigen::Vector3d(x, y, 0)) == true) {
          return true;
      } 
   }
   return false;
}

void PRM::drawMap() {
    for (int u_idx = 0; u_idx < vector_cnt; u_idx++) {
        for (auto p : graph[u_idx]) {
            int v_idx = p.first;
            Eigen::Vector3d u = V[u_idx];
            Eigen::Vector3d v = V[v_idx];
            map->updateMap(u, v);
        }
    }
    if (init_idx >= 0) {
        map->updateMap(V[init_idx]);
        map->updateMap(V[goal_idx]);
    }
}

void PRM::clearRoadmap() {
    map->clear();
}

PRM::~PRM()
{
    return;
}