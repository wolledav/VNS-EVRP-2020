//
// Created by David Woller on 16.03.20.
//

#ifndef EVRP_LS_OPERATORS_HPP
#define EVRP_LS_OPERATORS_HPP

#include <vector>

using namespace std;


bool twoStringMove(vector<int> &tour, int i, int j, int X, int Y);

bool twoString(vector<int> &tour, int X, int Y, bool firstImrpove);

double twoStringCostUpdate(vector<int> &tour, int i, int j, int X, int Y);

inline bool onePoint(vector<int> &tour, bool firstImprove) {
    return twoString(tour, 0, 1, firstImprove);
}

inline bool twoPoint(vector<int> &tour, bool firstImprove) {
    return twoString(tour, 1, 1, firstImprove);
}

inline bool threePoint(vector<int> &tour, bool firstImprove) {
    return twoString(tour, 1, 2, firstImprove);
}

void twoOptMove(vector<int> &tour, int i, int j);

bool twoOpt(vector<int> &tour, bool firstImprove);

double twoOptCostUpdate(vector<int> &tour, int i, int j);

bool AFSrealoc_one(vector<int> &tour, bool firstImprove);
bool AFSrealoc_more_than_one(vector<int> &tour, bool firstImprove);

#endif //EVRP_LS_OPERATORS_HPP
