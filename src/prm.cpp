#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <math.h>

#include <opencv2/opencv.hpp>
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
    graph = new vector<pair<int, double>>[n + 2];

    dijkstra = new Dijkstra();
    astar = new ASTAR();
    bfs = new BFS();
    dfs = new DFS();
}

void PRM::search() {
    double dx, dy, theta = 0;
    //stack<int> path = dijkstra->findPath(init_idx, goal_idx, vector_cnt, graph);
    //stack<int> path = astar->findPath(init_idx, goal_idx, vector_cnt, V, graph);
    //stack<int> path = bfs->findPath(init_idx, goal_idx, vector_cnt, graph);
    stack<int> path = dfs->findPath(init_idx, goal_idx, vector_cnt, graph);
    if (path.empty())
        return;

    Eigen::Vector3d pnt1 = V[path.top()];
    path.pop();
    while (!path.empty()) {
        Eigen::Vector3d pnt2 = V[path.top()];
        dx = pnt2(0) - pnt1(0);
        dy = pnt2(1) - pnt1(1);
        theta = atan(dy / dx);
        pnt2(3) = theta;
        map->updateMap(pnt1, pnt2);
        path.pop();
        pnt1 = pnt2;
    } 
    cv::waitKey(0);
}

void PRM::constructRoadmap() {
    int x, y, cnt = 0;
    double dx, dy = 0.0;
    double theta;
    MIN_QUEUE qn;
    Eigen::Vector3d pos;

    while (vector_cnt < n) {
        x = rand() % width;
        y = rand() % height;
        theta = -1;

        pos = Eigen::Vector3d(x, y, theta);

        if (!map->isCollision(pos) && checkDuplicateEdge(pos) == -1) {
            V[vector_cnt] = pos;
            vector_cnt += 1;
        }
        cnt += 1;
    }
    
    for (int i = 0; i < vector_cnt; i++) {
        int u_idx = i, v_idx = 0;
        double weight;
        int node_cnt = 0;
        qn = nearestNeighbors(V[i]);

        while (!qn.empty() && node_cnt < k) {
            pair<double, int> node = qn.top();
            qn.pop();
            v_idx = node.second;
            weight = node.first;
            if (!checkDuplicateEdge(u_idx, v_idx) && !map->isCollision(V[u_idx], V[v_idx])) {
                addEdge(u_idx, v_idx, weight);
                node_cnt += 1;
            }
        }
    }
}

MIN_QUEUE PRM::nearestNeighbors(Eigen::Vector3d q) {
    double dx, dy, dist = 0;
    MIN_QUEUE min_queue;
    for (int i = 0; i < n; i++) {
        if (q != V[i]) {
            dx = abs(q(0) - V[i](0));
            dy = abs(q(1) - V[i](1));
            dist = sqrt(pow(dx, 2) + pow(dy, 2));
            min_queue.push(make_pair(dist, i));
        }
    }
    return min_queue;
}

void PRM::removeNode(int u) {
    int v = graph[u].back().first;
    if (u < n) { // start node already existed in graph
        graph[u].pop_back();
    }
    else {
        V[u] = Eigen::Vector3d();
        graph[u] = vector<pair<int, double>>();
        vector_cnt -= 1;
    }
    graph[v].pop_back();
}


void PRM::setCoords(Eigen::Vector3d q_init, Eigen::Vector3d q_goal) {
    if (q_init(0) >= width || q_init(1) >= height || q_goal(0) >= width || q_goal(1) >= height) {
        cout << "outside map dimensions" << endl;
        return;
    }
    if (init_idx >= 0) {
        removeNode(init_idx);
        removeNode(goal_idx);
    }
    init_idx = addNode(q_init);
    if (init_idx == -1) {
        cout << "element not added" << endl;
        return;
    }
    goal_idx = addNode(q_goal);
    if (goal_idx == -1) {
        cout << "element not added" << endl;
        removeNode(goal_idx);
    }
}

int PRM::addNode(Eigen::Vector3d q_new) {
    double dist;
    Eigen::Vector3d q_curr;
    int u_idx; int v_idx; int dup_idx;
    pair<int, double> curr_config;
    MIN_QUEUE qn_new = nearestNeighbors(q_new); // configurations near init
    // add q_init to vector list
    dup_idx = checkDuplicateEdge(q_new);
    if (dup_idx == -1) {
        u_idx = vector_cnt;
        int node_cnt = 0;
        while (!qn_new.empty() && node_cnt < k) {
            v_idx = qn_new.top().second;
            dist = qn_new.top().first;
            qn_new.pop();
            q_curr = V[v_idx];

            if (!map->isCollision(q_new, q_curr)) {
                V[u_idx] = q_new;
                addEdge(u_idx, v_idx, dist);
                vector_cnt++;
                node_cnt += 1;
                break;
            }
        }
    }
    else
        u_idx = dup_idx;
    return u_idx;
}

void PRM::addEdge(int u_idx, int v_idx, double weight) {
    graph[u_idx].push_back(make_pair(v_idx, weight));
    graph[v_idx].push_back(make_pair(u_idx, weight));
}

int PRM::checkDuplicateEdge(Eigen::Vector3d pos) {
    for (int i = 0; i < vector_cnt; i++) {
        if (pos == (V[i])) {
            return i;
        }
    }
    return -1;
}

bool PRM::checkDuplicateEdge(int u, int v) {
    const int X_v(u);
    const int X_u(v);

    if (graph[u].size() == 0 || graph[v].size() == 0) {
        return false;
    }

    auto it_u = std::find_if(graph[u].begin(), 
                        graph[u].end(), 
                        [&X_v](const pair<int, double>& p)
                        {return p.first == X_v; });

    auto it_v = std::find_if(graph[v].begin(), 
                        graph[v].end(), 
                        [&X_u](const pair<int, double>& p)
                        {return p.first == X_u; });

    if (it_u == graph[u].end() && it_v == graph[v].end()) {
        return false; // duplicate entry
    }
    else {
        return true; // new entry
    }
}

bool PRM::bresenham(int x1, int y1, int x2, int y2) {
   int m_new = 2 * (y2 - y1); 
   int slope_error_new = m_new - (x2 - x1); 
   for (int x = x1, y = y1; x <= x2; x++) {
      if (map->isCollision(Eigen::Vector3d(x, y, 0)) == true) {
          return true;
      } 
      slope_error_new += m_new; 
      if (slope_error_new >= 0) { 
         y++; 
         slope_error_new  -= 2 * (x2 - x1); 
      } 
      if (map->isCollision(Eigen::Vector3d(x, y, 0)) == true) {
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
            //map->updateMap(u, v);
        }
    }
    if (init_idx >= 0) {
        map->updateMap(V[init_idx]);
        map->updateMap(V[goal_idx]);
    }
}

void PRM::clearCoords() {
    map->clear();
}

PRM::~PRM() {
    delete V;
    delete graph;
    return;
}