#include "ros/ros.h"
#include "amr/amr.h"
#include <math.h>
#include <stdlib.h>
#include <vector>

// std::vector<std::vector<float>> warehouse_specification = {{0.0 , 0.0},{2.4 , 0.0},{2.4 , 1.2},{2.4 , 2.46},{2.697 , 2.46},{2.697 , 1.2},
//             {3.6 , 1.2},{3.6 , 3.56},{0.0 , 3.56},{0.0 , 0.0}};

std::vector<std::vector<float>> warehouse_specification = {};
std::vector<std::vector<float>> transit_path = {};
std::vector<std::vector<float>> planned_path = {};