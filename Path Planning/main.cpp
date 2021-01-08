#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "prm.h"

using namespace std;

int main() {
    int n = 1000;
    int k = 10;
    PRM *prm = new PRM("map3.jpg", n, k);
    prm->roadmapConstruction();
    return 0;
}
