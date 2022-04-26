#include<iostream>
#include<cstdlib>


#include "EVRP.hpp"
#include "heuristics.hpp"
#include "stats.hpp"
#include "perturbations.hpp"

using namespace std;


/*initialiazes a run for your heuristic*/
void start_run(int r) {

    srand(r); //random seed
    init_evals();
    init_current_best();
    cout << "Run: " << r << " with random seed " << r << endl;
}

/*gets an observation of the run for your heuristic*/
void end_run(int r) {

    get_mean(r - 1, get_current_best()); //from stats.h
    cout << "End of run " << r << " with best solution quality " << get_current_best() << " total evaluations: "
         << get_evals() << endl;
    cout << " " << endl;
}

/*sets the termination conidition for your heuristic*/
bool termination_condition(void) {

    bool flag;
    if (get_evals() >= TERMINATION)
        flag = true;
    else
        flag = false;

    return flag;
}


/****************************************************************/
/*                Main Function                                 */
/****************************************************************/
int main(int argc, char *argv[]) {
    int run;
/*Step 1*/
    problem_instance = argv[1];       //pass the .evrp filename as an argument
    read_problem(problem_instance);   //Read EVRP from file from EVRP.h

/*Step 2*/
    open_stats();//open text files to store the best values from the 20 runs stats.h

    for (run = 1; run <= MAX_TRIALS; run++) {
/*Step 3*/
        start_run(run);
//Initialize your heuristic here
        initialize_heuristic(); //heuristic.h

/*Step 4*/
            run_heuristic();  //heuristic.h

// print_solution(best_sol->tour,best_sol->steps);
// check_solution(best_sol->tour,best_sol->steps);

/*Step 5*/
        end_run(run);  //store the best solution quality for each run
    }
/*Step 6*/
    close_stats(); //close text files to calculate the mean result from the 20 runs stats.h


//free memory
    free_stats();
    free_heuristic();
    free_EVRP();

    return 0;
}