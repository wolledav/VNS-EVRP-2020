#ifndef EVRP_HEURISTICS_HPP
#define EVRP_HEURISTICS_HPP

#include <vector>
#include "LS_operators.hpp"
#include "EVRP.hpp"

using namespace std;
using FunptrOperator = bool (*)(vector<int> &, bool);
using FunptrConstruction = vector<int> (*)();

struct solution{
  int *tour;	//this is what the fitness_evaluation function in EVRP.hpp will evaluate
  int id;
  double tour_length; //quality of the solution
  int steps; //size of the solution
  //the format of the solution is as follows:
  //*tour:  0 - 5 - 6 - 8 - 0 - 1 - 2 - 3 - 4 - 0 - 7 - 0
  //*steps: 12
  //this solution consists of three routes: 
  //Route 1: 0 - 5 - 6 - 8 - 0
  //Route 2: 0 - 1 - 2 - 3 - 4 - 0
  //Route 3: 0 - 7 - 0
};

extern solution *best_sol;

void initialize_heuristic();
void run_heuristic();
void free_heuristic();

void rvnd(vector<int> &tour, bool merge, bool firstImprove, vector<FunptrOperator> neighborhoods);
vector<int> ms_vns(bool merge, bool firstImprove, int p, double restart_ratio, vector<FunptrOperator> neighborhoods, vector<int>(*construction)(), void(*localSearch)(vector<int> &, bool, bool, vector<FunptrOperator>));



#endif //EVRP_HEURISTICS_HPP