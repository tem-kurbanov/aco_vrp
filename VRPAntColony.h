#include <random>
#include "Graph.h"
#include "ShrinkGraph.h"
#include "Label.h"
#include "SolverResult.h"

#ifndef ACO_VRP_VRPANTCOLONY_H
#define ACO_VRP_VRPANTCOLONY_H

// Complete terminal-graph ACO; path searches may cross intervening customers.
class VRPAntColony {
private:
    Graph graph;
    ShrinkGraph shrinked_graph;
    int depot = 0;
    int num_criticals;

    vector<pair<int, int>> shrinked_edge_nodes;
    vector<vector<int>> shrinked_edge_parameters;

    int k_cover = 24;
    int num_ants = 100;
    int iteration_limit = 20; // Consecutive iterations without a Pareto-archive update.
    int thread_count = 5;
    int max_vehicles = 0;
    unsigned int rng_seed = 0;

    // Intermediate cover graph, before terminal-to-terminal path construction.
    int num_cover_nodes = 0;
    int num_parameters = 2;

    int battery_capacity = 50000;

    vector<int> cover_id;
    vector<deque<int>> cover_edge_nodes;
    vector<vector<int>> cover_edge_params;
    vector<map<int, vector<int>>> outgoing_cover_edges;
    vector<map<int, vector<int>>> incoming_cover_edges;

    vector<int> max_parameter_values;

    // k counts road vertices, and the cover always retains depot/customers.
    void k_path_cover(int k);
    // Return nondominated cover-path labels for each reachable terminal.
    map<int, vector<LabelPtr>> t_kpc_mls(int source);

    bool check_domination(const vector<int>& v1, const vector<int>& v2) const;
    bool check_t_domination(const vector<int>& t, const vector<int>& v) const;
    bool t_discard(set<LabelPtr, lex_compare>& closed_labels, set<vector<int>>& t_set, vector<int>& new_vec);
    void update_t_set(set<vector<int>>& t_set, LabelPtr lab) const;

    vector<int> build_simple_timecons(int edge);
    vector<int> link_paths(vector<int>& current_timecons, vector<int>& suffix_timecons);

    int add_energy(int& current_energy, int& added_energy);
    bool validate_solution(const deque<int>& route_edges, const vector<int>& objectives,
                           int& route_count, string& reason);

    ShrinkGraph shrink_graph();

    pair<deque<int>, vector<int>> bicriterion_vrp_ant_crawl(vector<vector<double>>& pheromones, double& alpha, double& beta, double& lambda1, double& lambda2, double& exploration_probability, double& evaporation_rate, double& initial_pheromone, mt19937& rng);
    int bicriterion_ant_pick_edge(vector<int>& available_edges, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, mt19937& rng);

public:

    VRPAntColony(Graph original_graph, int num_goals, int ants = 100, int iterations = 20,
                 int threads = 5, int battery = 50000, int cover_k = 24, unsigned int seed = 0,
                 int vehicles = 0);

    string bicriterion_ant_vrp();
    SolverRunResult bicriterion_ant_vrp_result();

};

#endif //ACO_VRP_VRPANTCOLONY_H
