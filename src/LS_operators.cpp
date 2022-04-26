//
// Created by David Woller on 16.03.20.
//

#include <iostream>
#include <algorithm>
#include "LS_operators.hpp"
#include "utils.hpp"
#include "EVRP.hpp"
#include "constructions.hpp"

#define EPS 1e-9

using namespace std;

/*
 * Performs the twoString swap, given that the indices i, j and substring lengths X, Y are valid.
 */
bool twoStringMove(vector<int> &tour, int i, int j, int X, int Y) {
    // Invalid input for some inserts when X = 1, Y = 1 or vice versa
    if ((X == 0 || Y == 0) && i == j) {
        return false;
    }
    // Happens when X != Y
    if (j < i) {
        return twoStringMove(tour, j, i, Y, X);
    } else {
        /// This will be string (i + 1, ..., i + X) of the original path.
        vector<int> auxIPlus1ToIPlusX(X);
        /// This will be string (i + X + 1, ..., j) of the original path.
        vector<int> auxIPlusXPlus1ToJ(j - i - X);
        /// This will be string (j + 1, ..., j + Y) of the original path.
        vector<int> auxJPlus1ToJPlusY(Y);
        /// Save auxiliary substrings of the original path.
        for (int k = 0; k < auxIPlus1ToIPlusX.size(); ++k)
            auxIPlus1ToIPlusX[k] = tour[i + 1 + k];
        for (int k = 0; k < auxIPlusXPlus1ToJ.size(); ++k)
            auxIPlusXPlus1ToJ[k] = tour[i + X + 1 + k];
        for (int k = 0; k < auxJPlus1ToJPlusY.size(); ++k)
            auxJPlus1ToJPlusY[k] = tour[j + 1 + k];
        /// Update path.
        for (int k = 0; k < auxJPlus1ToJPlusY.size(); ++k)
            tour[i + 1 + k] = auxJPlus1ToJPlusY[k];
        for (int k = 0; k < auxIPlusXPlus1ToJ.size(); ++k)
            tour[i + Y + 1 + k] = auxIPlusXPlus1ToJ[k];
        for (int k = 0; k < auxIPlus1ToIPlusX.size(); ++k)
            tour[j + Y - X + 1 + k] = auxIPlus1ToIPlusX[k];
        return true;
    }
}

/*
 * Attempts to perform twoString for all possible indices i and j, given the substring lengths X and Y.
 * Returns true if the tour is improved (best improvement).
 */
bool twoString(vector<int> &tour, int X, int Y, bool firstImprove) {
//    cout << "2string" << endl;
    vector<int> tmpTour;
    vector<int> bestTour;
    tmpTour.reserve(tour.size());
    bestTour.reserve(tour.size());

    double impBest = EPS;
    bool improved = false;
    for (int i = 0; i < tour.size() - X; ++i) {
        if (get_evals() > STOP_CNT or (firstImprove and improved)) break;
        for (int j = (X == Y) ? i + X : 0; j < tour.size() - Y; ++j) {
            if (get_evals() > STOP_CNT or (firstImprove and improved)) break;
            bool cond1 = i + X == j and (X == 0 or Y == 0);
            bool cond2 = j + Y == i and (X == 0 or Y == 0);
            if (not cond1 and not cond2 and ((j - i >= X) || (i - j >= Y))) {
                // More efficient in terms of distance requests (validity check -> cost update)
//                tmpTour = tour;
//                twoStringMove(tmpTour, i, j, X, Y);
//                if (isValidTour(tmpTour)) {
//                    auto impCurr = j < i ? twoStringCostUpdate(tour, j, i, Y, X) : twoStringCostUpdate(tour, i, j, X, Y);
//                    if (impCurr > impBest) {
//                        impBest = impCurr;
//                        bestTour = tmpTour;
//                        improved = true;
//                    }
//                }
                // Faster setup (cost update -> validity check)
                auto impCurr = j < i ? twoStringCostUpdate(tour, j, i, Y, X) : twoStringCostUpdate(tour, i, j, X, Y);
                if (impCurr > impBest) {
                    tmpTour = tour;
                    twoStringMove(tmpTour, i, j, X, Y);
                    if (isValidTour(tmpTour)) {
                        impBest = impCurr;
                        bestTour = tmpTour;
                        improved = true;
                    }
                }

            }
        }
    }
    if (improved) {
        tour = bestTour;
    }
    return improved;
}

/*
 * Returns tour fitness difference after performing twoStringMove(tour, i, j, X, Y).
 * Positive value means fitness improvement.
 */
double twoStringCostUpdate(vector<int> &tour, int i, int j, int X, int Y) {
    auto last = tour.size() - 1;

    // Evaluate cost of removed edges
    double cut1 = get_distance(tour[i], tour[i+1]); // cut after i
    double cut2 = X != 0 ? get_distance(tour[i + X], tour[i + X + 1]) : 0; // cut after i + X
    double cut3 = (j != last and j != i + X) ? get_distance(tour[j], tour[j + 1]) : 0; // cut after j
    double cut4 = (Y != 0 and j + Y != last) ? get_distance(tour[j + Y], tour[j + Y + 1]) : 0; // cut after j + Y

    double removed = cut1 + cut2 + cut3 + cut4;

    // Evaluate cost of added edges
    double add1 = Y != 0 ? get_distance(tour[i], tour[j + 1]) : get_distance(tour[i], tour[i + X + 1]); // edge added after i
    double add2 = 0; // edge added after inserted Y block
    if (Y != 0) add2 = j != i + X ? get_distance(tour[j + Y], tour[i + X + 1]) : get_distance(tour[j + Y], tour[i + 1]);
    double add3 = 0; // edge added after j
//    if (j != i + X) add3 = X != 0 ? get_distance(tour[j], tour[i + 1]) : get_distance(tour[j], tour[j + Y + 1]);
    if (j != i + X) {
        if (X != 0) {
            add3 = get_distance(tour[j], tour[i + 1]);
        } else {
            if (j != last and j + Y != last) add3 = get_distance(tour[j], tour[j + Y + 1]);
        }
    }
    double add4 = (X != 0 and j + Y != last) ? get_distance(tour[i + X], tour[j + Y + 1]) : 0; // edge added after inserted X block

    double added = add1 + add2 + add3 + add4;

    return removed - added;
}

/*
 * Performs the 2-opt: reverses the order of the elements at positions i to j (both included)
 */
void twoOptMove(vector<int> &tour, int i, int j) {
    std::reverse(tour.begin() + i, tour.begin() + j + 1);
}

/*
 * Attempts to perform all possible 2-opt moves.
 * Returns true if the tour is improved
 */
bool twoOpt(vector<int> &tour, bool firstImprove) {
    vector<int> tmpTour;
    vector<int> bestTour;
    tmpTour.reserve(tour.size());
    bestTour.reserve(tour.size());
    bool improved = false;
    double impBest = EPS;
    for (int i = 1; i < tour.size() - 1; i++) {
        if (get_evals() > STOP_CNT or (firstImprove and improved)) break;
        for (int j = i + 1; j < tour.size() - 1; j++) {
            if (get_evals() > STOP_CNT or (firstImprove and improved)) break;
            // More efficient in terms of distance requests (validity check -> cost update)
//            tmpTour = tour;
//            twoOptMove(tmpTour, i, j);
//            if (isValidTour(tmpTour)) {
//                double impCurr = twoOptCostUpdate(tour, i, j);
//                if (impCurr > impBest) {
//                    impBest = impCurr;
//                    bestTour = tmpTour;
//                    improved = true;
//                }
//            }
            // Faster setup (cost update -> validity check)
            auto impCurr = twoOptCostUpdate(tour, i, j);
            if (impCurr > impBest) {
                tmpTour = tour;
                twoOptMove(tmpTour, i, j);
                if (isValidTour(tmpTour)) {
                    impBest = impCurr;
                    bestTour = tmpTour;
                    improved = true;
                }
            }

        }
    }
    if (improved) {
        tour = bestTour;
    }
    return improved;
}

double triangle_adition(int afs, int b, int c){
    return get_distance(b, afs) + get_distance(afs, c) - get_distance(b, c);
}

double triangle_adition2(int afs, int b, int c, double dist_bc){
    return get_distance(b, afs) + get_distance(afs, c) - dist_bc;
}


//AFS reallocation for tour with more than one AFS
double AFSrealoc_more(vector<int> &tour){
    //cout << "AFSrealoc more" << endl;
    //Basically just testing AFS allocation on reversed tour
    double tour_dist = 0;
    double f_tour_dist = 0;
    double r_tour_dist = 0;

    //Get reverse route
    vector<int> r_tour;
    int num_c_orig = 0;
    for(int i = (tour.size() - 1); i >= 0; i--){
        int node = tour.at(i);
        if(!is_charging_station(node)){
            r_tour.push_back(node);
            num_c_orig++;
        }
    }

    //Front
    vector<int> f_tour = r_tour;
    std::reverse(f_tour.begin(),f_tour.end());
    
    // ZGA has better results than SSF
    // but doesnt work with one customer that needs more AFS
    // this usually happens with initOneRouteEach

    f_tour = tsp2evrp_zga_mini(f_tour);
    //f_tour = tsp2evrp_ssf(f_tour);

    //Back
    //AFS alloc
    r_tour = tsp2evrp_zga_mini(r_tour);
    //r_tour = tsp2evrp_ssf(r_tour);

    for(int i = 0; i < tour.size()-1; i++){
        int n1 = tour.at(i);
        int n2 = tour.at(i+1);
        tour_dist += get_distance(n1, n2);
    }

    for(int i = 0; i < f_tour.size()-1; i++){
        int n1 = f_tour.at(i);
        int n2 = f_tour.at(i+1);
        f_tour_dist += get_distance(n1, n2);
    }

    for(int i = 0; i < r_tour.size()-1; i++){
        int n1 = r_tour.at(i);
        int n2 = r_tour.at(i+1);
        r_tour_dist += get_distance(n1, n2);
    }

    int num_c_new = 0;

    if(tour_dist <= f_tour_dist && tour_dist <= r_tour_dist){
        //cout << "Keeping original tour with original AFS" << endl;
        return 0;
    }

    if(f_tour_dist <= r_tour_dist){
        //cout << "Returns computed front tour with new AFS" << endl;

        for(int i = 0; i < f_tour.size(); i++){
            int node = f_tour.at(i);
            if(!is_charging_station(node)){
                num_c_new++;
            }
        }

        if(num_c_orig != num_c_new){
            //ZGA probably lost a customer
            return 0;
        }

        double improvement = tour_dist - f_tour_dist;

        if (improvement > EPS){
        tour = f_tour;
        }

        return improvement;
    }
    else{

        for(int i = 0; i < r_tour.size(); i++){
            int node = r_tour.at(i);
            if(!is_charging_station(node)){
                num_c_new++;
            }
        }

        if(num_c_orig != num_c_new){
            //ZGA probably lost a customer
            return 0;
        }

        //cout << "Returns reversed tour with new AFS" << endl;
        std::reverse(r_tour.begin(),r_tour.end());

        double improvement = tour_dist - r_tour_dist;
                
        if (improvement > EPS){
        tour = r_tour;
        }

        return improvement;
    }

}

bool lost_customers(vector<int> test_tour, int orig_number){
    int new_number = 0;
    for(int i = 0; i < test_tour.size(); i++){
        int node = test_tour.at(i);
        if( !is_charging_station(node)){
            new_number++;
        }
    }

    if(orig_number != new_number){
        //cout << "Tour lost customers. Orig: " << orig_number << " new: " << new_number << endl<< endl;
        return true;
    }

    return false;
}

//Tries to improve AFSs location for tours with one AFS
//Returns the improvement value (positive)
//Returns 0 if no improvement found
double AFSrealoc_one_afs(vector<int> &tour, int original_afs_at){
    //cout << "AFSrealoc one" << endl;

    //Compute additional distance created by the AFS in the path generated in front direction
    int afs = tour.at(original_afs_at);
    int b = tour.at(original_afs_at - 1);
    int c = tour.at(original_afs_at + 1);
    //cout << "afs:" << afs << " b:" << b << " c: " << c << endl;
    double additional_dist_original = triangle_adition(afs, b, c);
    //cout << "add orig:" << additional_dist_original << endl;

    //Get route without charging stations
    //Reverse order is here just because it was implemented first
    vector<int> r_tour;
    int n_of_cust_orig = 0;
    for(int i = (tour.size() - 1); i >= 0; i--){
        int node = tour.at(i);
        if( !is_charging_station(node)){
            r_tour.push_back(node);
            n_of_cust_orig++;
        }
    } 

    //Front
    vector<int> f_tour = r_tour;
    std::reverse(f_tour.begin(),f_tour.end());
    f_tour = tsp2evrp_zga_mini(f_tour);
    //f_tour = tsp2evrp_ssf(f_tour);

    if(lost_customers(f_tour, n_of_cust_orig)){
        cout << "ZGA lost some customers" << endl;
    }

    int front_afs_at = 0;
    int n_of_front_afs = 0;
    for(int i = 1; i < f_tour.size() - 1; i++){
        int node = f_tour.at(i);
        if(is_charging_station(node)){
            n_of_front_afs++;
            front_afs_at = i;
            //cout << "front node " << node <<" at " << i  << endl;
            //*break;
        }
    }

    //* this might be unnecessary if we breake the previous cycle correctly
    if(n_of_front_afs > 1){
        //cout << "More than one AFS in reverted tour" << endl;
        return 0;
    }
    else if(n_of_front_afs < 1){
        //cout << "AFS was unnecessary" << endl;
        double improvement = additional_dist_original;

        if (improvement > EPS){
            tour = f_tour;
        }
        return improvement;
    }


    //Back
    r_tour = tsp2evrp_zga_mini(r_tour);
    //r_tour = tsp2evrp_ssf(r_tour);
    if(lost_customers(r_tour, n_of_cust_orig)){
        cout << "ZGA lost some customers" << endl;
    }

    int back_afs_at = 0;
    int n_of_back_afs = 0;
    for(int i = 1; i < r_tour.size() - 1; i++){
        int node = r_tour.at(i);
        if(is_charging_station(node)){
            n_of_back_afs++;
            back_afs_at = i;
            //*break;
        }
    }

    //* this might be unnecessary if we breake the previous cycle correctly
    if(n_of_back_afs > 1){
        //cout << "More than one AFS in reverted tour" << endl;
        return 0;
    }
    else if(n_of_back_afs < 1){
        double improvement = additional_dist_original;

        if (improvement > EPS){
            tour = r_tour;
        }
        return improvement;
    }

    //Compute additional distance created by the AFS in the path generated in forward direction
    afs = f_tour.at(front_afs_at);
    b = f_tour.at(front_afs_at - 1);
    c = f_tour.at(front_afs_at + 1);
    //cout << "afs:" << afs << " b:" << b << " c: " << c << endl;
    double additional_dist_front_afs = triangle_adition(afs, b, c);
    //cout << "add front:" << additional_dist_original << endl;

    //Compute additional distance created by the AFS in the path generated in backward direction
    afs = r_tour.at(back_afs_at);
    b = r_tour.at(back_afs_at - 1);
    c = r_tour.at(back_afs_at + 1);
    //cout << "afs:" << afs << " b:" << b << " c: " << c << endl;
    double additional_dist_back_afs = triangle_adition(afs, b, c);


    int back_pos_in_orig = tour.size() - back_afs_at - 1;

    if(front_afs_at == back_pos_in_orig && r_tour.at(back_afs_at) == tour.at(front_afs_at)){
        //cout << "Same AFS at the same edge, no change" << endl;
        return 0;
    }
    else if(abs(front_afs_at - back_pos_in_orig) == 1){
        //cout << "AFSs at neighbouring edges, choosing AFS with lower additional distance" << endl;
        //cout << "Front afs at: " << front_afs_at << " adi dist: " << additional_dist_front_afs << endl;
        //cout << "Back afs at: " << back_pos_in_orig << "(" << back_afs_at << ")" << " adi dist: " << additional_dist_back_afs << endl;
        
        if(additional_dist_original <= additional_dist_front_afs 
            && additional_dist_original <= additional_dist_back_afs){
            //Keeps original tour with original AFS
            //cout << "i0 " << endl;
            return 0;
        }
        else if(additional_dist_front_afs <= additional_dist_back_afs){
            //cout << "Returns forward tour with computed AFS" << endl;
            double improvement = additional_dist_original - additional_dist_front_afs;

            if (improvement > EPS){
                tour = f_tour;
            }
            //cout << "i1 " << improvement << endl;
            return improvement;
        }
        else{
            //cout << "Returns reversed tour with new AFS" << endl;
            std::reverse(r_tour.begin(),r_tour.end());
            double improvement = additional_dist_original - additional_dist_back_afs;
                    
            if (improvement > EPS){
                tour = r_tour;
            }
            //cout << "i2" << endl;
            return improvement;
        }
    }

    //Switch from lower to higher (more intuitive)
    int front = back_pos_in_orig;
    int back = front_afs_at;
    double minAdd = 0;
    bool front_smaller = false; //remember in case no value is better than minAdd
    bool back_smaller = false;
    bool original_smaller = false;
    double improvement = -1;

    //Get smallest distance addition as base value
    if(additional_dist_original <= additional_dist_front_afs 
            && additional_dist_original <= additional_dist_back_afs){
        minAdd = additional_dist_original;
        original_smaller = true;
    }
    else if(additional_dist_front_afs <= additional_dist_back_afs){
        front_smaller = true;
        minAdd = additional_dist_front_afs;
    }
    else{
        back_smaller = true;
        minAdd = additional_dist_back_afs;
    }
    //cout << "front: " << front << " back: " << back << " minAdd: " << minAdd << endl;


    //HERE IS WHERE THE REAL COMPARISON STARTS,... LOL :-D
    vector<int> clean_tour;

    //Create a clean tour without unnecessary afs (or depots)
    for(int i = 0; i < f_tour.size(); i++){
        int node = f_tour.at(i);
        if(i != back){
            clean_tour.push_back(node);
        }
        else{
            //remove this one
            //cout << "X";
        }
        //cout << node << " ";
    }
    //cout << endl;

    int best_index = -1;
    int best_afs = -1;
    for(int i = front; i < back - 1; i++){
        double bi = clean_tour.at(i);
        double ci = clean_tour.at(i + 1);
        double dist_bc = get_distance(bi, ci);

        //Triangular addition with potential AFS
        for (int j = 0; j < ACTUAL_PROBLEM_SIZE; j++) {
            if (is_charging_station(j)) {
                int p_afs = j;
                double addi = triangle_adition2(p_afs, bi, ci, dist_bc);
                if(addi < minAdd){
                    //cout << "Improvement bi " << bi << " ci " << ci << " afs " << afs <<endl;
                    minAdd = addi;
                    best_index = i;
                    best_afs = p_afs;
                }
            }
        }
    }

    improvement = additional_dist_original - minAdd;

    //No improvement
    if(improvement <= EPS){
        //cout << "Keeping original tour with original AFS" << endl;

        //cout << "i3 " << improvement << endl;
        return improvement;
    }

    //If nothing smaller was found
    if(best_index == -1){

        if(original_smaller){
            //cout << "Original was the smallest anyway" << endl;
            //cout << "i35" << endl;
            return 0;
        }

        if(front_smaller){
            improvement = additional_dist_original - additional_dist_front_afs;
            
            if (improvement > EPS){
            tour = f_tour;
            }

            //cout << "i4 " << improvement << endl;
            return improvement;
        }

        if(back_smaller){
            //cout << "Returns reversed tour with new AFS" << endl;
            improvement = additional_dist_original - additional_dist_back_afs;
                    
            if (improvement > EPS){
            tour = r_tour;
            }

            //cout << "i5" << endl;
            return improvement;
        }
    }


    //SOMETHING BETTER HAS BEEN FOUND
    //RETURN UPDATED TOUR

    //cout << "best_index: " << best_index << endl; 
    //Insert best AFS
    clean_tour.insert(clean_tour.begin() + best_index + 1, best_afs);
    if (improvement > EPS){
        if(lost_customers(clean_tour, n_of_cust_orig)){
            cout << "CLEAN TOUR lost some customers" << endl;
        }
        tour = clean_tour;
    }

    //New tour
    //for(int i = 0; i < clean_tour.size(); i++){
    //    int node = clean_tour.at(i);
        //cout << node << " ";
    //}
    //cout << endl;

    //cout << "minAdd: " << minAdd << endl; 
    //cout << " Affs n."<< clean_tour.at(best_index + 1);
    //cout << " added in between " << clean_tour.at(best_index) << " and " << clean_tour.at(best_index + 2) << endl;

    //cout << "i6 " << improvement << endl;
    return improvement;
}

//Separates the tour to subtours separated by individual depo visits
//Tries to realocate AFSs in individual subtours to improve lenght
//
//firstImprove does nothing - for now
//Could be improved, if we know where the depo is just for Charging and where it is for capacity
bool AFSrealoc_common(vector<int> &tour, bool firstImprove, bool more_than_one){
    //cout << "AFSrealoc common start."<< endl;
    double evals_start = get_evals();
    bool eval_ok = true;

    bool did_it_improve = false;

    vector<vector<int>> subtours;
    vector<int> sub_tour;
    bool first = true;

    //Separate to subtours
    for(int i = 0; i < tour.size(); i++){
        int node = tour.at(i);
        if(node == 0 && !first){
            sub_tour.push_back(node);
            subtours.push_back(sub_tour);
            sub_tour.clear();
        }
        sub_tour.push_back(node);
        first = false;
    }

    //Get number of AFSs in each subtour
    //(If one afs, get its position)
    vector<int> sub_tour_afs_n;
    vector<int> sub_tour_afs_at;
    int n_of_one_AFS_subtours = 0;
    for(int j = 0; j < subtours.size(); j++){
        //cout << "Sub tour "<< j << endl;
        auto st = subtours.at(j);
        int n_of_afs = 0;
        int afs_at = 0;
        for(int i = 1; i < st.size() - 1; i++){
            int node = st.at(i);
            if(is_charging_station(node)){
                n_of_afs++;
                afs_at = i;
                //cout << node << " ";
            }
        }
        sub_tour_afs_n.push_back(n_of_afs);
        sub_tour_afs_at.push_back(afs_at);
        if(n_of_afs == 1){
            n_of_one_AFS_subtours++;
        }
        //cout << endl;
        //cout << "n of afs " << n_of_afs << endl;
    }

    bool first_update = false;
    //double max_ev = 0;
    double total_improvement = 0;
    //create new vector of subtours (possibly) with modified subtours
    vector<vector<int>> subtours_new;
    for(int j = 0; j < subtours.size(); j++){
        if(get_evals() > TERMINATION - 25){
            eval_ok = false;
        }
        //double evals_tmp = get_evals();
        auto st = subtours.at(j);
        auto st_orig = subtours.at(j);
        int n_of_afs = sub_tour_afs_n.at(j);

        double improvement = 0;

        //Modify only subtours with one AFS
        // - because it's easier
        if (n_of_afs == 1 && !more_than_one && eval_ok){
            //cout << "S tour "<< j << endl;

            //Depending on firstImprove:
            //Either TRY updating all, or TRY from random number upwards
            //If the random one fails, try the next,...
            //Does not try the previous ones
            if(!first_update || !firstImprove){
                int afs_at = sub_tour_afs_at.at(j);
                improvement = AFSrealoc_one_afs(st, afs_at);
                if(improvement > EPS){
                    first_update = true;
                }
            }
        }
        else if(n_of_afs > 1 && more_than_one && eval_ok){
            //cout << "S tour "<< j << endl;

            if(!first_update || !firstImprove){
                int afs_at = sub_tour_afs_at.at(j);
                improvement = AFSrealoc_more(st);
                if(improvement > EPS){
                    first_update = true;
                }
            }
        }

        if(improvement > EPS){
            //cout << "AFS all improvement " << improvement << endl;
            //cout << "Sub-tour " << j << " improved by " << improvement << endl;
            total_improvement += improvement;
            subtours_new.push_back(st);
        }
        else{
            subtours_new.push_back(st_orig);
        }

        //if(max_ev < (get_evals() - evals_tmp)){
        //    max_ev = get_evals() - evals_tmp;
        //}
    }

    //Join all subtours in one
    vector<int> evrp_tour;
    for(auto& st:subtours_new){
        evrp_tour.insert(evrp_tour.end(), st.begin(), st.end());
    }

    //Delete duplicated depots
    int last  = -1;
    for(int i = 0; i < evrp_tour.size(); i++ ){
        if(last == 0 && evrp_tour.at(i) == 0){
            evrp_tour.erase (evrp_tour.begin()+i);
            i--;
        }
        last = evrp_tour.at(i);
    }

    if (total_improvement > EPS){
        tour = evrp_tour;
        did_it_improve = true;
    }

    //if(more_than_one){
    //    cout << "AFSrealoc more Evals used: " << get_evals() - evals_start << "  max: " << max_ev << endl;
    //}
    //else{
    //    cout << "AFSrealoc one Evals used: " << get_evals() - evals_start << "  max: " << max_ev  << endl;
    //}
    //cout << "AFS all total improvement " << total_improvement << endl;
    //cout << "AFSrealoc finished. Total improvement: " << total_improvement << endl;
    return did_it_improve;
}

bool AFSrealoc_one(vector<int> &tour, bool firstImprove){
    //cout << "Out AFSrealoc _one" << endl;
    return AFSrealoc_common(tour, firstImprove, false);
}

bool AFSrealoc_more_than_one(vector<int> &tour, bool firstImprove){
    // cout << "Out AFSrealoc _more" << endl;
    return AFSrealoc_common(tour, firstImprove, true);
}

/*
 * Returns tour fitness difference after performing twoOptMove(tour, i, j).
 * Positive value means fitness improvement.
 */
double twoOptCostUpdate(vector<int> &tour, int i, int j) {
    double cut1 = get_distance(tour[i - 1], tour[i]); // cut before i
    double cut2 = get_distance(tour[j], tour[j + 1]); // cut after j

    double removed = cut1 + cut2;

    double add1 = get_distance(tour[i - 1], tour[j]); // edge added after i - 1
    double add2 = get_distance(tour[i], tour[j + 1]); // edge added before j + 1

    double added = add1 + add2;


    return removed - added;
}

