//
// Created by David Woller on 16.03.20.
//

#ifndef EVRP_UTILS_HPP
#define EVRP_UTILS_HPP

#include <vector>

using namespace std;

int getRemainingLoad(vector<int> evrpTour);
double getRemainingBattery(vector<int> evrpTour);
int getClosestAFS(int node);
int getReachableAFSClosestToGoal(int cur, int goal, double battery);
bool isValidTour(vector<int> tour);
void mergeAFSs(vector<int> &tour);

vector<std::string> tokenize(std::string const &str, char delim);
void printTour(vector<int> &tour);
int getRandomAmongAvailable(unsigned availableCount, const vector<bool> &available);



//Variables and functions for dbca
const int dbca_OUTLIER = -2;
const int dbca_NOT_CLASSIFIED = -1;

class dbca_Point {
public:
    int id;
    int neighbours_count = 0;
    int cluster_ID;
    vector<int> adjacentPoints;
};
void density_connected(int current_p, int cluster, int min_pt, vector<dbca_Point> &points);


#endif //EVRP_UTILS_HPP
