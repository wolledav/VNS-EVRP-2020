//
// Created by David Woller on 16.03.20.
//

#ifndef EVRP_CONSTRUCTIONS_HPP
#define EVRP_CONSTRUCTIONS_HPP

#include <vector>
#include <chrono>

using namespace std;

// TSP 2 EVRP functions
vector<int> tsp2evrp_zga_relaxed(vector<int> tspTour);

// Other functions
vector<vector<vector<int>>> dbca();
vector<int> clarke_wright(bool capacitated, bool clusters = false, vector<int> node_list = vector<int>());
vector<int> init_from_dbca(int method);

//Noncapacitated zga for subtours with one AFS
vector<int> tsp2evrp_zga_mini(vector<int> tspTour);

// Standalone EVRP constructions
inline vector<int> initDbcaClarkeZga() {
    return init_from_dbca(4);
}

#endif //EVRP_CONSTRUCTIONS_HPP
