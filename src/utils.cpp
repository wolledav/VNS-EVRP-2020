//
// Created by David Woller on 16.03.20.
//

#include <cfloat>
#include <iostream>
#include <sstream>
#include <random>
#include "utils.hpp"
#include "EVRP.hpp"


/*
 * Returns remaining loading capacity after performing the given evrp tour.
 */
int getRemainingLoad(vector<int> evrpTour) {
    int load = 0;
    for (auto node:evrpTour) {
        if (node == 0) {
            load = MAX_CAPACITY;
        } else {
            load -= get_customer_demand(node);
        }
    }
    return load;
}

/*
 * Returns remaining fuel capacity after performing the given evrp tour.
 */
double getRemainingBattery(vector<int> evrpTour) {
    double battery = 0;
    for (int i = 0; i < evrpTour.size(); i++){
        int cur = evrpTour[i];
        if (i > 0) {
            int prev = evrpTour[i - 1];
            battery -= get_energy_consumption(prev, cur);
        }
        if (is_charging_station(cur)) {
            battery = BATTERY_CAPACITY;
        }
    }
    return battery;
}

/*
 * Returns the closest AFS to the given node.
 */
int getClosestAFS(int node) {
    auto minDist = DBL_MAX;
    int closest;
    for (int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
        if (is_charging_station(i) and i != node) {
            double dist = get_distance(node, i);
            if (dist < minDist) {
                minDist = dist;
                closest = i;
            }
        }
    }
    return closest;
}

/*
 * Returns the closest AFS to the given goal, that is reachable from the current position.
 */
int getReachableAFSClosestToGoal(int cur, int goal, double battery) {
    auto minDist = DBL_MAX;
    int closest = -1;
    for (int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
        if (is_charging_station(i) && i != cur && battery >= get_energy_consumption(cur, i)) {
            double dist = get_distance(i, goal);
            // cout << "Station: " << i << ", distance: " << dist << endl;
            if (dist < minDist) {
                minDist = dist;
                closest = i;
            }
        }
    }
    return closest;
}

/*
 * Checks validity of given evrp tour.
 * Terminates right after reaching first invalid node.
 */
bool isValidTour(vector<int> tour) {
    int load = 0;
    double battery = 0;
    int custCnt = 0;
    for (int i = 0; i < tour.size(); i++) {
        int cur = tour[i];
        // Load check
        if (cur == 0) {
            load = MAX_CAPACITY;
        } else {
            load -= get_customer_demand(cur);
            if (load < 0) return false;
        }
        // Battery check
        if (i > 0) {
            int prev = tour[i - 1];
            battery -= get_energy_consumption(prev, cur);
            if (battery < 0) return false;
        }
        if (is_charging_station(cur)){
            battery = BATTERY_CAPACITY;
        } else{
            custCnt++;
        }
    }
    // Closed tour check
    if ((tour[0] != 0) || (tour[tour.size() - 1] != 0)) {
        return false;
    }
    if (custCnt < NUM_OF_CUSTOMERS) {
        return false;
    }
    return true;
}

/*
 * Sequentially traverses tour and removes neighboring indentical nodes.
 */
void mergeAFSs(vector<int> &tour) {
    int i = 0;
    while (i != tour.size() - 1) {
        if (tour[i] == tour[i+1]) { // remove element at index i+1
            tour.erase(tour.begin() + i + 1);
        } else {
            i++;
        }
    }
}

/*
 * Tokenize a string str to a vector
 */
vector<string> tokenize(string const &str, char delim) {
    std::stringstream ss(str);
    vector<string> out;
    std::string s;
    while (std::getline(ss, s, delim)) {
        out.push_back(s);
    }
    return out;
}

/*
 * Prints tour to cout
 */
void printTour(vector<int> &tour) {
    for (auto n:tour) cout << n << " ";
    cout << endl;
}

int getRandomAmongAvailable(unsigned availableCount, const vector<bool> &available) {
//    default_random_engine generator(chrono::system_clock::now().time_since_epoch().count());
    default_random_engine generator(rand());
    uniform_int_distribution<int> distribution(0,availableCount - 1);
    auto r = distribution(generator);
    for (unsigned i = 0; i < available.size(); ++i) {
        if (available[i]) {
            if (r == 0) {
                return i;
            }
            --r;
        }
    }
    return -1;

}

void density_connected(int current_p, int cluster, int min_pt, vector<dbca_Point> &points) {
    points.at(current_p).cluster_ID = cluster;
    if(points.at(current_p).neighbours_count < min_pt) return;
        
    for(auto& next:points.at(current_p).adjacentPoints) {
        if(points.at(next).cluster_ID != dbca_NOT_CLASSIFIED &&
           points.at(next).cluster_ID != dbca_OUTLIER) continue;

        density_connected(next, cluster, min_pt, points);
    }
}
