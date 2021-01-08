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
            vector_cnt += 1;
        }
    }

    for (int i = 0; i < vector_cnt; i++) {
        vector<pair<int, float>> qn = nearestNeighbors(V[i]);
        int unique_cnt = 0;
        int dup_cnt = 0;
        cout << "neighbors: " << qn.size() << endl;
        for (auto curr_neighbor : qn) {
            int u_idx = i;
            int v_idx = curr_neighbor.first;
            float weight = curr_neighbor.second;
            
            bool duplicate = checkDuplicateEdge(u_idx, v_idx);
            if (duplicate == false) {
                bool collision = checkCollision(V[u_idx], V[v_idx]);
                if (collision == false) {
                    pair<int, float> node_u(u_idx, weight);
                    pair<int, float> node_v(v_idx, weight);
                    graph[u_idx].push_back(node_v);
                    graph[v_idx].push_back(node_u);
                    map->updateMap(V[u_idx](0), V[u_idx](1), V[v_idx](0), V[v_idx](1));
                    unique_cnt += 1;
                }
                else {
                    cout << "collision: " << "(" << V[u_idx](0) << ", " <<  V[u_idx](1) << " ) -> " << "(" << V[v_idx](0) << ", " <<  V[v_idx](1) << " )" << endl;
                    //map->updateMap(V[u_idx](0), V[u_idx](1), V[v_idx](0), V[v_idx](1));
                }
            }
            else {
                dup_cnt++;
            }
        }
        cout << "duplicate vertices: " << dup_cnt << endl;
        cout << "unique vertices: " << unique_cnt << endl;
    }
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

bool PRM::checkCollision(Eigen::Vector2d pnt0, Eigen::Vector2d pnt1) {
    // Bresenham's line algorithm

    int x1 = pnt0(0);
    int y1 = pnt0(1);
    int x2 = pnt1(0);
    int y2 = pnt1(1);

   int m_new = 2 * (y2 - y1); 
   int slope_error_new = m_new - (x2 - x1); 
   for (int x = x1, y = y1; x <= x2; x++) 
   { 
      if (checkCollision(Eigen::Vector2d(x, y)) == true) {
          return true;
      } 
  
      // Add slope to increment angle formed 
      slope_error_new += m_new; 
  
      // Slope error reached limit, time to 
      // increment y and update slope error. 
      if (slope_error_new >= 0) 
      { 
         y++; 
         slope_error_new  -= 2 * (x2 - x1); 
      } 
   }
   return false;
}

bool PRM::checkCollision(Eigen::Vector2d q) {
    bool res;
    int val = map->getCell(q(0), q(1));

    if (val == 255) // no collision = white
        res = false;
    else if (val == 0) // collision = black
        res = true;

    return res;
}

typedef pair<float, int> pi;

vector<pair<int, float>> PRM::nearestNeighbors(Eigen::Vector2d q) {
    float dx, dy, dist = 0;
    priority_queue<pi, vector<pi>, greater<pi>> min_queue;
    for (int i = 0; i < n; i++) {
        if (q != V[i]) {
            dx = abs(q(0) - V[i](0));
            dy = abs(q(1) - V[i](1));
            dist = sqrt(pow(dx, 2) + pow(dy, 2));
            min_queue.push(make_pair(dist, i));
        }
    }
    //cout << "(" << q(0) << ", " << q(1) << ")" << endl << "++++++++++++++" << endl;
    vector<pair<int, float>> neighbor_idx;
    for (int i = 0; i < k; i++) {
        pair<float, int> node = min_queue.top();
        neighbor_idx.push_back({node.second, node.first});
        //cout << "(" << V[node.second](0) << ", " << V[node.second](1) << ")" << " = " << node.first << endl;
        min_queue.pop();
    }
    //cout << "______________" << endl;
    return neighbor_idx;
}

PRM::~PRM()
{
    return;
}