//
// Created by David Woller on 26.03.20.
//

#include <iostream>
#include "floydWarshall.hpp"
#include "EVRP.hpp"

using namespace std;

/*
 * This constructor initializes the planning graph from global list AFSs.
 */
floydWarshall::floydWarshall(int nV) : nV(nV) {
    next = vector<vector<int>>(nV, vector<int>(nV, -1));
    dist = vector<vector<int>>(nV, vector<int>(nV, INF));
    for (int i = 0; i < nV; i++) {
        for (int j = i; j < nV; j++) {
            if (i == j) {
                dist[i][j] = 0;
                next[i][j] = j;
            } else {
                int start = AFSs[i];
                int goal = AFSs[j];
                double consumption = get_energy_consumption(start, goal);
                get_distance(start, goal); // Called here just to increase evals, so that the usage is 100% legal
                if (consumption <= BATTERY_CAPACITY) {
                    dist[i][j] = consumption;
                    dist[j][i] = consumption;
                    next[i][j] = j;
                    next[j][i] = i;
                }
            }
        }
    }
}

/*
 * This constructor can be used for initialization by any graph.
 */
floydWarshall::floydWarshall(vector<vector<int>> &graph) : dist(graph) {
    next = vector<vector<int>>(nV, vector<int>(nV, -1));
    nV = graph.size();
    for (int i = 0; i < nV; i++) {
        for (int j = 0; j < nV; j++) {
            next[i][j] = j;
        }
    }
}

/*
 * Plans all pairs of shortest paths, run just once.
 */
void floydWarshall::planPaths() {
    int i, j, k;
    for (k = 0; k < nV; k++) {
        for (i = 0; i < nV; i++) {
            for (j = 0; j < nV; j++) {
                if (dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next[i][j] = next[i][k];
                }
            }
        }
    }
}

void floydWarshall::printMatrix(vector<vector<int>> &matrix) {
    for (auto i = 0; i < nV; i++) {
        for (auto j = 0; j < nV; j++) {
            cout << matrix[i][j] << " ";
        }
        cout << endl;
    }
}

/*
 * Returns shortest path from u to v.
 * If afsIds = true, returns AFS ids as stored in the global AFSs vector, otherwise returns internal ids used by the floydWarshall instance.
 */
vector<int> floydWarshall::getPath(int u, int v, bool afsIds) {
    vector<int> path;
    if (next[u][v] == -1) {
        return path;
    }
    path.push_back(u);
    while (u != v) {
        u = next[u][v];
        path.push_back(u);
    }
    if (afsIds) {
        for (auto &n:path) {
            n = AFSs[n];
        }
    }

    return path;
}
