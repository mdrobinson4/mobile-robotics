#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <stack>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
class BFS {
public:
    BFS();
    ~BFS();
    stack<int> findPath(int, int, int, vector<pair<int, double>>[]);
    stack<int> reconstructPath(vector<int> came_from, int current, int);
private:
};