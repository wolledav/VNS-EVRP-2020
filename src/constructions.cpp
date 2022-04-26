//
// Created by David Woller on 16.03.20.
//

#include <set>
#include <cfloat>
#include <iostream>
#include <assert.h>
#include <algorithm>

#include "constructions.hpp"
#include "EVRP.hpp"
#include "utils.hpp"

using namespace std;

bool addAndCheckLastN(int node, bool reset = false) {

    //N is 4
    static int lastN[4];
    static int nextLastNIndex = 0;
    static int counter = 0;
    if (reset) {
        for (int i = 0; i < 4; i++) {
            lastN[i] = -1;
        }
        counter = 0;
    }

    counter++;
    bool check = true;
    if (counter > 4) {
        check = false;
        for (int i = 0; i < 2; i++) {
            int index1 = i % 4;
            int index2 = (i + 2) % 4;
            if (lastN[index1] != lastN[index2]) {
                check = true;
                break;
            }
        }
    }
    if (check) {
        lastN[nextLastNIndex] = node;
        nextLastNIndex = (nextLastNIndex + 1) % 4;
        return true;
    } else {
        cout << "cycle detected: " << lastN[0] << ", " << lastN[1] << ", " << lastN[2] << ", " << lastN[3] << endl;
        return false;
    }
}

bool get_to_depot_possibly_through_afss(vector<int> &evrpTour) {
    bool canReachDepot = getRemainingBattery(evrpTour) - get_energy_consumption(evrpTour.back(), 0) >= 0;
    if (canReachDepot) {
        evrpTour.push_back(0);
    } else {
        // NOTE: can be optimized
        int closestAFS = getClosestAFS(evrpTour.back());
        vector<int> afsSubpath = fw.getPath(afsIdMap[closestAFS], afsIdMap[0], true);
        evrpTour.insert(evrpTour.end(), afsSubpath.begin(), afsSubpath.end());
    }
    return addAndCheckLastN(0);
}

/*
 * Generate a valid E-VRP tour from a given TSP tour over customers only.
 * Based on Zhang, Gajpal and Appadoo (2017) - relaxed to allow routes to depot via AFS
 */
vector<int> tsp2evrp_zga_relaxed(vector<int> tspTour) {
    vector<int> evrpTour;
    int nextId = 0; // index of the next customer in the tspTour

    // Start at depot
    evrpTour.push_back(0);

    //bool satisfiable = false;
    // (1) All customers served? -> NO
    while (nextId != tspTour.size()) {
        // (2) Next customer satisfiable? -> NO
        if (getRemainingLoad(evrpTour) < get_customer_demand(tspTour[nextId])) {
            // NOTE: CHANGE TO ZGA
            bool check = get_to_depot_possibly_through_afss(evrpTour);
            if (!check) break;
            // (2) Next customer satisfiable? -> YES
        } else {
            // (3) Can return to the closest AFS via the next customer? -> YES
            // NOTE: CHANGE TO ZGA
            int closestAFSToGoal = getClosestAFS(tspTour[nextId]);
            double remainingBattery = getRemainingBattery(evrpTour);
            double energyToNext = get_energy_consumption(evrpTour.back(), tspTour[nextId]);
            double nextToAFS = get_energy_consumption(tspTour[nextId], closestAFSToGoal);
            if (remainingBattery - energyToNext >= nextToAFS) {
                evrpTour.push_back(tspTour[nextId]);
                nextId++;
                if (!addAndCheckLastN(nextId - 1)) break;
                // (3) Can return to the closest AFS via the next customer? -> NO
            } else {
                // (4) Can reach nearest AFS? -> NO
                // NOTE: this can be optimized, too...
                int closestAFS = getReachableAFSClosestToGoal(evrpTour.back(), tspTour[nextId], getRemainingBattery(evrpTour));
                bool canReach = getRemainingBattery(evrpTour) - get_energy_consumption(evrpTour.back(), closestAFS) >= 0;
                assert(canReach);
                // (4) Can reach nearest AFS? -> YES
                evrpTour.push_back(closestAFS);
                if (!addAndCheckLastN(closestAFS)) break;
            }
        }
    }
    // (1) All customers served? -> YES
    get_to_depot_possibly_through_afss(evrpTour);
    return evrpTour;
}

/*
 * Non-capacitated zga for subtours with one AFS
 * Adds AFSs
 */
vector<int> tsp2evrp_zga_mini(vector<int> tspTour) {
    vector<int> evrpTour;

    int nextId = 0; // index of the next customer in the tspTour

    addAndCheckLastN(-1, true);

    // Start at depot
    evrpTour.push_back(0);

    //bool satisfiable = false;
    // (1) All customers served? -> NO
    while (nextId != tspTour.size()) {
            // (3) Can return to the closest AFS via the next customer? -> YES
            // NOTE: CHANGE TO ZGA
            int closestAFSToGoal = getClosestAFS(tspTour[nextId]);
            double remainingBattery = getRemainingBattery(evrpTour);
            double energyToNext = get_energy_consumption(evrpTour.back(), tspTour[nextId]);
            double nextToAFS = get_energy_consumption(tspTour[nextId], closestAFSToGoal);
            if (remainingBattery - energyToNext >= nextToAFS) {
                evrpTour.push_back(tspTour[nextId]);
                nextId++;
                if (!addAndCheckLastN(tspTour[nextId - 1])) break;
                // (3) Can return to the closest AFS via the next customer? -> NO
            } else {
                // (4) Can reach nearest AFS? -> NO
                // NOTE: this can be optimized, too...
                int closestAFS = getReachableAFSClosestToGoal(evrpTour.back(), tspTour[nextId], getRemainingBattery(evrpTour));
                bool canReach = getRemainingBattery(evrpTour) - get_energy_consumption(evrpTour.back(), closestAFS) >= 0;
                assert(canReach);
                // (4) Can reach nearest AFS? -> YES
                evrpTour.push_back(closestAFS);
                //asf_id = closestAFS;
                if (!addAndCheckLastN(closestAFS)) break;
            }

    }

    // (1) All customers served? -> YES
    get_to_depot_possibly_through_afss(evrpTour);

    return evrpTour;
}

/*Clarke wright initialization 
 * Returns path over customers and depot, respecting max capacity
 * AFSs not not implemented yet
 */
vector<int> clarke_wright(bool capacitated, bool clusters, vector<int> node_list) {
    // list of unused customers
    set<int> unusedCustomers;

    //Compute on separate cluster of customer nodes
    if (clusters){
        for (int node:node_list) {
            if (!is_charging_station(node)) {
                unusedCustomers.insert(node);
            }
        }
    }else {
        //Compute on all customer nodes
        for (int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
            if (!is_charging_station(i)) {
                unusedCustomers.insert(i);
            }
        }
    }

    vector<vector<int>> subtours;
    vector<int> subtours_length;

    while(unusedCustomers.size() > 0){
        vector<int> subtour;
        int remaining_capacity = MAX_CAPACITY;
        double length = 0;
        //get the one furthest from depo from the remaining customers
        auto maxDist = 0;
        int furthest;
        for (int cand:unusedCustomers){
            double dist = get_distance(0, cand);
            if (dist > maxDist){
                maxDist = dist;
                furthest = cand;
            }
        }
        subtour.push_back(furthest);
        remaining_capacity -= get_customer_demand(furthest);
        unusedCustomers.erase(furthest);

        double dist_front = maxDist;
        double dist_back = maxDist;

        bool enough_capacity = true;
        //vector<touple> pairs;

        //Add closest customers to the group until capacity is full
        while(enough_capacity) {
            enough_capacity = false;
            //The lower the better
            auto distImprovement = DBL_MAX;
            int closest;
            int neigbour;
            
            int front = subtour.front();
            int back = subtour.back();
            bool at_front = false;
            double dist_to_depo;

            for (int cand:unusedCustomers){
                if((get_customer_demand(cand) <= remaining_capacity ) || !capacitated){
                    enough_capacity = true;
                    //double dist_front = get_distance(front, cand);
                    //double dist_back = get_distance(back, cand);

                    //Get improvement compared to a direct path depo-candidate-depo
                    //dist saved = ( new distance ) - ( original distance )
                    //original distance = distance( original -> depo + depo -> candidate + candidate -> depo)
                    //new distance = distance( original -> candiate -> depo)
                    double dist_candidate_depo = get_distance(0, cand);


                    double dist_saved_front = get_distance(front, cand) - dist_candidate_depo - dist_front;
                    double dist_saved_back = get_distance(back, cand) - dist_candidate_depo - dist_back;

                    if (dist_saved_front < distImprovement){
                        at_front = true;
                        distImprovement = dist_saved_front;
                        closest = cand;
                        dist_to_depo = dist_candidate_depo;
                        //neigbour = cust;
                    }

                    if (dist_saved_back < distImprovement){
                        at_front = false;
                        distImprovement = dist_saved_back;
                        closest = cand;
                        dist_to_depo = dist_candidate_depo;
                        //neigbour = cust;
                    }

                }
            }

            if(!enough_capacity){
                //subtours.push_back(subtour);
                //subtours_length.push_back(length);
                break;
            }


            if(at_front){
                dist_front = dist_to_depo;
                subtour.insert(subtour.begin(),closest);
            }
            else{
                dist_back = dist_to_depo;
                subtour.push_back(closest);
            }


            //pairs.push_back(neigbour, closest);
            remaining_capacity -= get_customer_demand(closest);
            unusedCustomers.erase(closest);

            if(unusedCustomers.size() == 0){
                enough_capacity = false;
                break;
            }
            //could be useful, to insert appropriate number of AFSs
            //length += minDist;
        }

        subtours.push_back(subtour);
        //subtours_length.push_back(length);
    }

    vector<int> tmp;
    if(capacitated){
        tmp.push_back(0);
        for(auto tour:subtours){
            for(int customer:tour){
                tmp.push_back(customer);
            }
            tmp.push_back(0);
        }
    } else{
        tmp = subtours.at(0);
    }

    return tmp;
}

/*
 * Density-based spatial clustering
 * Creates and returns several sets of clusters of vertices
 * returns: vector<vector<vector<int>>>
 * Sets for different parameters-> Sets -> Vertices
 */
vector<vector<vector<int>>> dbca() {
    //cout << "DBCA" << endl;
    double reach = BATTERY_CAPACITY/get_energy_per_unit();
    //cout << "Reach: " << reach << endl;

    //Variables for cluster variations
    // - Should probably be based on max reach on battery, or some other relevant parameter

    //Init density tresholds (minimal radius to connect a cluster)
    vector<double> epss;
    //Minimal number of neigbouring points for a cluster core
    vector<int> min_pts;

    //TODO - experiment with proper ranges of these parameters
    epss.push_back(reach/(2));
    epss.push_back(reach/(3));
    for (int i = 2; i < 6; i++){ 
        epss.push_back(reach/(i*2));
    }
    epss.push_back(reach/(15));
    epss.push_back(reach/(20));
    

    for (int i = 2; i < 6; i++){ 
        min_pts.push_back(i);
    }

    //Prepare output: sets for different parameters-> Sets -> Vertices
    vector<vector<vector<int>>> cluster_sets;

    //Iterate through all param combinations

    //Iterate through epsilons
    for (int x = 0; x < epss.size(); x++){ 
        double eps = epss.at(x);
        //cout << "////////////////////////////////////////" << endl;
        //cout << endl << "Eps: " << eps << endl;
        dbca_Point points_base[ACTUAL_PROBLEM_SIZE];

        //Determine neighbours for each vertex
        for(int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
            points_base[i].cluster_ID = dbca_NOT_CLASSIFIED;
            for(int j = 0; j < ACTUAL_PROBLEM_SIZE; j++) {
                if(i == j) continue;
                if(get_distance(i, j) <= eps) {
                    points_base[i].neighbours_count++;
                    points_base[i].adjacentPoints.push_back(j);
                }
            }
            if(points_base[i].neighbours_count < 1){
                //cout << "No neighbours " << i << endl;
            }
        }

        //Iterate through min points per cluster core
        for (int y = 0; y < min_pts.size(); y++){
            int min_pt = min_pts.at(y);
            //cout << endl << "////" << "Params min_pt: " << min_pt << ", eps: " << eps << "////" << endl;

            int cluster_idx = -1;
            
            //Copy the computed point descriptors, so we dont have to compute them again
            vector<dbca_Point> points;
            vector<vector<int>> cluster_set;
            for(int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
                points.push_back(points_base[i]);
            }

            //Classify vertices in clusters
            for(int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
                //Skip if the point is already clasified
                if(points.at(i).cluster_ID != dbca_NOT_CLASSIFIED) continue;
                
                //If the point is a core
                if(points.at(i).neighbours_count >= min_pt) {
                    cluster_idx++;
                    //cout << "Core of cluster " << cluster_idx << " is node " << i << endl;

                    //Mark all unclassified nodes that are density-connected
                    density_connected(i, cluster_idx, min_pt, points);
                } else {
                    //Not a core
                    points.at(i).cluster_ID = dbca_OUTLIER;
                    //cout << " " << i ;
                }
            }

            //cout << endl << "dbca_OUTLIER sorting" << endl;
            for(int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
                if(points.at(i).cluster_ID != dbca_OUTLIER) continue;

                double minDist = DBL_MAX;
                int min_node_id = - 2;

                for(int j = 0; j < ACTUAL_PROBLEM_SIZE; j++) {
                    if(i == j) continue;
                    if(points.at(j).cluster_ID == dbca_OUTLIER) continue;

                    double dist = get_distance(i, j);
                    if(dist < minDist){
                        minDist = dist;
                        min_node_id = j;
                    }
                }

                if(min_node_id == -2){
                    cluster_idx++;
                    points.at(i).cluster_ID = cluster_idx;
                    //cout << "No clusters, returns all points in one" << endl;
                }else {
                    points.at(i).cluster_ID = points.at(min_node_id).cluster_ID;
                    //cout << "dbca_OUTLIER " << i << " added to cluster "<< points.at(i).cluster_ID << endl;
                }
            }
            //cout << endl;

            //Get a vector of clusters
            cluster_set.resize(cluster_idx + 1);
            for(int i = 0; i < ACTUAL_PROBLEM_SIZE; i++) {
                if(points.at(i).cluster_ID != dbca_OUTLIER) {
                    cluster_set[points.at(i).cluster_ID].push_back(i);
                }
            }

            //Remove clusters with only AFSs (or depot)
            for(int i = 0; i < cluster_set.size(); i++){
                //cout << "Cluster " << i << endl;
                bool has_customer = false;
                for(auto& next:cluster_set.at(i)){
                //    cout << (next) << " " ;
                    if(!is_charging_station(next)){
                        has_customer = true;
                    }
                }

                //If the cluster doesnt have a customer it is useles
                if(!has_customer){
                    cout << "No customer in cluster "<< endl;
                    cluster_set.erase (cluster_set.begin()+i);
                }
                //cout << endl;
            }

            //cout << endl;
            //Add depots
            for(int i = 0; i < cluster_set.size(); i++){
                //cout << "Cluster " << i << endl;
                bool has_depo = false;
                for(auto& next:cluster_set.at(i)){
                //    cout << (next) << " " ;
                    if(next == DEPOT){
                        has_depo = true;
                    }
                }

                //If the cluster doesnt have depot, add it
                if(!has_depo){
                    cluster_set.at(i).push_back(DEPOT);
                }
                //cout << endl;
            }

            //Do not add duplicates
            bool has_duplicate = false;
            for(auto& c:cluster_sets){
                if(c == cluster_set) has_duplicate = true;
            }

            if(has_duplicate) {
                //cout << "Has duplicate" << endl;
                //cout << "c size:" << cluster_set.size() << endl;
                continue;
            }
            else{
                //cout << "No duplicate" << endl;
            }

            //See if it works with clusters
            //TODO - remove this line after dbca is tested
            //if(cluster_set.size() == 1) continue;


            //Push back cluster sets for these parameters
            cluster_sets.push_back(cluster_set);

        }
    }

    //Sets for different parameters-> Sets -> Vertices
    //vector<vector<vector<int>>>
    return cluster_sets;
}

/*
 * Density-based spatial clustering
 * creates several cluster sets with different distributions
 * constructs evrp turs for each cluster set and returns the best evrpTour
 */
vector<int> init_from_dbca(int method) {
    cout << "Init DCBA " << endl;

                switch (method) {
                case 1:
                    //NN_SSF
                    cout << "NN_SSF" << endl;
                    break;
                case 2:
                    //NN_ZGA
                    cout << "NN_ZGA" << endl;
                    break;
                case 3:
                    //CLARKE_SSF
                    cout << "CLARKE_SSF" << endl;
                    break;
                case 4:
                    //CLARKE_ZGA
                    cout << "CLARKE_ZGA" << endl;
                    break;
                case 5:
                    //MST_SSF
                    cout << "MST_SSF" << endl;
                    break;
                case 6:
                    //MST_ZGA
                    cout << "MST_ZGA" << endl;
                    break;
                default:
                    cout << "UNIDENTIFIED PARAMETER" << endl;
            }
    
    //Create several sets of clusters of vertices
    //Creation based on eps and min_p parameters ()
    auto cluster_sets = dbca();

    cout << "DCBA select. C-sets size:" << cluster_sets.size() << endl;
    //Create an evrp tour for each cluster set
    vector<vector<int>> evrp_tours;
    for(int x = 0; x < cluster_sets.size(); x++){
        auto cluster_set = cluster_sets.at(x);
        vector<int> evrp_tour;
        //cout << endl << "Cluster set n. " << x << " start" << endl;


        //Create a tour for each cluster
        for(int y = 0; y < cluster_set.size(); y++){
            auto cluster = cluster_set.at(y);
            //cout << "c size " << cluster.size() << endl;

            vector<int> initTour;
            vector<int> tmp_evrp_tour;
            switch (method) {
                case 4:
                    //CLARKE_ZGA
                    initTour = clarke_wright(true, true, cluster);
                    tmp_evrp_tour = tsp2evrp_zga_relaxed(initTour);
                    break;
                default:
                    cout << "UNIDENTIFIED PARAMETER" << endl;
            }

            //NOT MODIFIED yet
            //initMstTspTour()

            //cout << "evrp done" << endl;
            //Merge tours from individual clusters
            evrp_tour.insert(evrp_tour.end(), tmp_evrp_tour.begin(), tmp_evrp_tour.end());

        }
        //cout << "Cluster set n. " << x << " done" << endl;

        //Delete duplicated depots
        int last  = -1;
        for(int i = 0; i < evrp_tour.size(); i++ ){
            if(last == 0 && evrp_tour.at(i) == 0){
                evrp_tour.erase (evrp_tour.begin()+i);
                i--;
            }
            last = evrp_tour.at(i);
        }
        
        //store created tours
        evrp_tours.push_back(evrp_tour);
    }


    //Get the best tour
    double minEval = DBL_MAX;
    vector<int> bestTour;
    for(auto& tour:evrp_tours){
        double eval = fitness_evaluation(tour);
        if(eval < minEval){
            minEval = eval;
            bestTour = tour;
        }
    }

    cout << "DBCA best tour fitness: " << minEval << endl;

    return bestTour;
}


