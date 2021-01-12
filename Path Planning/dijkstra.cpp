#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <inttypes.h>
#include <limits>

#include "dijkstra.h"

using namespace std;

stack<int> Dijkstra::findPath(int init, int goal, int cnt, vector<pair<int, double>> graph[]) {
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    vector<double> dist(cnt);
    vector<int> prev(cnt);
    stack<int> path;

    vector<bool> done(cnt, false);
    
    for (int i = 0; i < cnt; i++) {
        dist[i] = numeric_limits<double>::max();
        prev[i] = -1;
    }

    pq.push(make_pair(0, init));
    dist[init] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) {
            cout << "found goal" << endl;

            while (u != -1) {
                path.push(u); 
                u = prev[u]; 
            }
            return path;
        }

        vector<pair<int, double>>::iterator it;
        for (it = graph[u].begin(); it != graph[u].end(); it++) {
            int v = (*it).first;
            double weight = (*it).second;
            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.push(make_pair(dist[v], v));
            }
            
        }
        done[u] == true;
    }
    return stack<int> ();
}