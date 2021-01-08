#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

class MAP
{
private:
    cv::Mat input;
    cv::Mat modInput;
public:
    MAP(cv::Mat input);
    int getCell(int u, int v);
    void updateMap(int u, int v);
    void updateMap(int, int, int, int);
    ~MAP();
};