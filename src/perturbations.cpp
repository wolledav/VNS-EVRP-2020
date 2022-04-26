//
// Created by David Woller on 06.04.20.
//

#include <iostream>
#include <random>
#include <algorithm>
#include "perturbations.hpp"
#include "utils.hpp"
#include "constructions.hpp"


/*
 * Performs the generalized double bridge move - splits the tour to p+1 subtours, then recconects them in
 * random order and orientation.
 * Adapted from JM code.
 * Modified tour is not guaranteed to be a valid EVRP tour!
 */
void generalizedDoubleBridge(vector<int> &tour, unsigned p) {
    auto n = tour.size();
    p = (p + 1 > n) ? n - 1 : p;

    /// First randomly choose p different positions on the path.
//    default_random_engine generator(chrono::system_clock::now().time_since_epoch().count());
    default_random_engine generator(rand());
    auto distribution = uniform_int_distribution<int>(0,n-2);
    std::vector<bool> throwsBool(n, false);
    for (unsigned i = 0; i < p; ++i) {
        int throwI;
        do {
            throwI = distribution(generator);
        } while (throwsBool[throwI]);
        throwsBool[throwI] = true;
    }

    /// Then split the path to (p + 1) strings :
    ///     pathStrings[0]: < 0, 1, ... , throw(1) >,
    ///     pathStrings[1]: < throw(1) + 1, ... , throw(2) >
    ///     ...
    ///     pathStrings[p - 1]: < throw(p - 1) + 1, ... , throw(p) >,
    ///     pathStrings[p]: < throw(p) + 1, ... , n >
    std::vector<std::vector<int>> pathStrings{}; // these are indices
    for (unsigned i = 0; i < n; ++i) {
        std::vector<int> pathString{};
        while (!throwsBool[i] && i < n - 1) {
            pathString.emplace_back(i++);
        }
        pathString.emplace_back(i);
        pathStrings.push_back(pathString);
    }

    /// Create new path that starts with the first string.
    std::vector<int> newPositions(pathStrings[0]);

    /// Create roulette and shuffle it to get an order in which the rest of the strings will be connected.
    std::vector<int> roulette = std::vector<int>(pathStrings.size());
    for (unsigned i = 0; i < roulette.size(); ++i) roulette[i] = i;
    std::shuffle(roulette.begin() + 1, roulette.end(), generator);

    /// Connect the strings to get the new path.
    for (unsigned k = 1; k < pathStrings.size(); ++k) {

        /// Pick a next string to be connected to the new path.
        auto &nextString = pathStrings[roulette[k]];

        /// Flip a coin.
        auto side = uniform_int_distribution<unsigned>(0, 1)(generator);
        if (side == 0) {

            /// If side == 0, then connect the next string in ascending order.
            for (int i : nextString) {
                newPositions.push_back(i);
            }

        } else {

            /// If side == 1, then connect the next string in descending order.
            for (unsigned i = 0; i < nextString.size(); ++i) {
                newPositions.push_back(nextString[nextString.size() - i - 1]);
            }
        }
    }

    /// Now get the path's vertices.
    std::vector<int> newPath(n);
    for (unsigned j = 0; j < n; ++j) {
        newPath[j] = tour[newPositions[j]];
    }


    // Fix the perturbed tour
    newPath = tsp2evrp_zga_relaxed(newPath);
    mergeAFSs(newPath);

    tour = newPath;
}
