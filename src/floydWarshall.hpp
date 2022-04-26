//
// Created by David Woller on 26.03.20.
// Taken from https://www.programiz.com/dsa/floyd-warshall-algorithm and modified.
// O(n^3) time complexity, but plans just once.
//

#ifndef EVRP_FLOYDWARSHALL_HPP
#define EVRP_FLOYDWARSHALL_HPP

#include <vector>

#define INF 0x3f3f3f3f

using namespace std;

class floydWarshall {
    int nV; // graph size
    vector<vector<int>> dist; // matrix of shortest distances; dist[i][j] ~ distance from i to j
    vector<vector<int>> next; // next[i][j] ~ next node on the shortest path from i to j

public:
    floydWarshall() = default;
    explicit floydWarshall(int nV);
    explicit floydWarshall(vector<vector<int>> &graph);
    void planPaths();
    vector<int> getPath(int u, int v, bool afsIds);
    void printMatrix(vector<vector<int>> &matrix);
};


#endif //EVRP_FLOYDWARSHALL_HPP
