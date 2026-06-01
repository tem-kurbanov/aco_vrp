#include "Graph.h"
#include "SVRPAntColony.h"
#include "VRPAntColony.h"
#include <iostream>
#include <random>
#include <fstream>

using namespace std;

void read_graph(vector<pair<int, int>>& edge_points, vector<pair<int, int>>& edge_parameters) {
    string line;

    getline(cin, line);
    getline(cin, line);

    while (getline(cin, line, ',')) {
        int start = stoi(line);

        getline(cin, line, ',');
        int end = stoi(line);

        getline(cin, line, ',');
        int p1 = stoi(line);

        getline(cin, line, '\n');
        int p2 = stoi(line);

        edge_points.emplace_back(start, end);
        edge_parameters.emplace_back(p1, p2);
    }
}

int main(int argc, char *argv[]) {
    int num_nodes = 1748;
    int num_goals = stoi(argv[1]);

    std::random_device rd;     // only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> c1(0, num_nodes-1); // guaranteed unbiased

    vector<pair<int, int>> edge_points;
    vector<pair<int, int>> edge_parameters;

    read_graph(edge_points, edge_parameters);

//    int depot = 285;

    int num_tests = 10;
    for (int i = 0; i < num_tests; ++i) {

        int depot = c1(rng);
//        int depot = 107593;

        vector<bool> is_goal(num_nodes, false);
        set<int> goals;
        int num_activated = 0;
        while (num_activated < num_goals) {
            int new_goal = c1(rng);
            if (!is_goal[new_goal] && new_goal != depot) {
                goals.insert(new_goal);
                is_goal[new_goal] = true;
                num_activated += 1;
            }
        }

//        set<int> goals({6207,6292,7943,9001,9423,9430,9461,10696,11431,14829,16213,18060,18322,19065,20467,21916,25699,26540,28355,29897,32376,34938,35249,41583,42687,46418,46843,47112,47632,50997,51913,52331,53593,57718,60375,61063,62290,63345,65056,65090,65573,67584,68886,71969,76082,76373,76945,77767,79101,83660,84546,85105,86520,86561,87362,87683,88499,89967,90579,90638,94276,95656,100250,102252,102331,102830,103897,104929,106490,106911,108599,108964,109235,109985,112491,115194,116031,116142,116419,117172,117455,118909,119231,120883,121998,124827,126048,126621,127418,129504,129769,129789,131251,131708,132096,132272,132604,134553,135452,137823});

        Graph graph(num_nodes, edge_points, edge_parameters, depot, goals);

//        ofstream goal_file;
//        goal_file.open("../test_berlin_goals10.csv", ios::app);
//        goal_file << depot << "  ";
//        for (int g: goals) {
//            goal_file << g << ",";
//        }
//        goal_file << "\n";
//        goal_file.close();

        num_goals = goals.size();

        SVRPAntColony sant(graph, num_goals);
        string svrp_answer = sant.bicriterion_ant_svrp();
//        for (auto &sol: svrp_solutions) {
//            cout << "(" << sol.second[0] << "; " << sol.second[1] << "), ";
//        }
//        cout << endl;

        VRPAntColony ant(graph, num_goals);
        string vrp_answer = ant.bicriterion_ant_vrp();
//        for (auto &sol: vrp_solutions) {
//            cout << "(" << sol.second[0] << "; " << sol.second[1] << "), ";
//        }
//        cout << endl;

        cout << svrp_answer << "|" << vrp_answer << endl;
//        cout << svrp_answer << endl;

    }

    return 0;
}
