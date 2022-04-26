#include<iostream>
#include<limits.h>

#include "heuristics.hpp"
#include "EVRP.hpp"
#include "utils.hpp"
#include "perturbations.hpp"
#include "constructions.hpp"

using namespace std;


solution *best_sol;   //see heuristic.hpp for the solution structure

/*initialize the structure of your heuristic in this function*/
void initialize_heuristic(){

    best_sol = new solution;
    best_sol->tour = new int[4*NUM_OF_CUSTOMERS];
    best_sol->id = 1;
    best_sol->steps = 0;
    best_sol->tour_length = INT_MAX;
}

/*implement your heuristic in this function*/
void run_heuristic(){
    initMyStructures();

    auto construction = initDbcaClarkeZga;
    std::vector<FunptrOperator> selectedOperators{
            AFSrealoc_one,
            AFSrealoc_more_than_one,
            twoOpt,
            onePoint,
            twoPoint,
            threePoint
    };

    bool merge = true;
    bool firstImprove = false;
    int p = 2;
    double restart_ratio = 0.35;

    auto evrpTour = ms_vns(merge, firstImprove, p, restart_ratio, selectedOperators, construction, rvnd);

    full_validity_check(evrpTour);

    // Fill best_sol structure
    for (int i = 0; i < evrpTour.size(); i++) {
        best_sol->tour[i] = evrpTour[i];
    }
    best_sol->steps = evrpTour.size();
    best_sol->tour_length = fitness_evaluation(evrpTour);
}

/*free memory structures*/
void free_heuristic(){

  delete[] best_sol->tour;

}

void rvnd(vector<int> &tour, bool merge, bool firstImprove, vector<FunptrOperator> neighborhoods) {
    int i = 0;

    // nlSize ~ number of available neighborhoods
    auto nlSize = static_cast<unsigned>(neighborhoods.size());

    // available[i] == true ~ neighborhood i is available
    std::vector<bool> available(neighborhoods.size(), true);

    // Iterate until there are available neighborhoods
    while (nlSize > 0) {
        // Pick neighborhood index randomly among available neighborhoods
        auto neighborhoodIdx = getRandomAmongAvailable(nlSize, available);
        // Attempt to improve the tour in the selected neighborhood
        if ((*neighborhoods[neighborhoodIdx])(tour, firstImprove)) {
            // If success, then set all neighborhoods to be available again
            nlSize = static_cast<unsigned>(neighborhoods.size());
            std::fill(available.begin(), available.end(), true);
            if (merge) {
                mergeAFSs(tour);
            }
            double a = get_evals();
            double b = TERMINATION;
            // cout << "RVND iter: " << ++i << ", evals: " << get_evals() << ", fitness: " << fitness_evaluation(tour) << ", progress: " << a/b << endl;
        } else {
            // If not success, then make the current neighborhood unavailable and decrease nlSize
            available[neighborhoodIdx] = false;
            --nlSize;
        }
    }
}

vector<int>
ms_vns(bool merge, bool firstImprove, int p, double restart_ratio, vector<FunptrOperator> neighborhoods, vector<int> (*construction)(),
       void (*localSearch)(vector<int> &, bool, bool, vector<FunptrOperator>)) {
    int n = ACTUAL_PROBLEM_SIZE;
    int vns_restarts = n*restart_ratio;
    int vns_cnt = 0;
    vector<int> very_best;
    double very_best_score;
    // cout << "vns_restarts: " << vns_restarts << endl;

    // Outer loop
    while (get_evals() < STOP_CNT) {
        auto best = construction();

        double best_score = fitness_evaluation(best);
        if (very_best.size() == 0){
            very_best = best;
            very_best_score = best_score;
        }

        // Attempt at most vns_restarts iters. of VNS
        while (vns_cnt < vns_restarts && get_evals() < STOP_CNT) {
            auto current = best;
            generalizedDoubleBridge(current, p);
            localSearch(current, merge, firstImprove, neighborhoods);
            double current_score = fitness_evaluation(current);

            double a = get_evals();
            double b = TERMINATION;
            cout << "Non-improving VNS cnt: " << vns_cnt << ", current: " << current_score << ", best: " << best_score << ", very best: " << very_best_score << ", progress: " << a/b << endl;


            if (current_score < best_score) {
                vns_cnt = 0;
                best = current;
                best_score = current_score;
            } else {
                vns_cnt++;
            }
        }
        // Update very best before VNS restart
        if (get_evals() < STOP_CNT) cout << "VNS full restart\n";
        // cout << "vns_cnt: " << vns_cnt << endl;
        vns_cnt = 0;
        if (best_score < very_best_score) {
            very_best = best;
            very_best_score = best_score;
        }
    }

    return very_best;
}

