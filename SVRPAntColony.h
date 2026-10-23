#include <random>
#include "Graph.h"
#include "ShrinkGraph.h"
#include "Label.h"
#include "SolverResult.h"

#ifndef ACO_VRP_ANTCOLONY_H
#define ACO_VRP_ANTCOLONY_H

// Sparse terminal-graph ACO with optional customer-crossing path candidates.
class SVRPAntColony {
private:
    Graph graph;
    ShrinkGraph shrinked_graph;
    int num_criticals;
    int depot = 0;

    vector<pair<int, int>> shrinked_edge_nodes;
    vector<vector<int>> shrinked_edge_parameters;

    int k_cover = 24;
    int num_ants = 100;
    int iteration_limit = 20; // Consecutive iterations without a Pareto-archive update.
    int thread_count = 5;
    int max_vehicles = 0;
    unsigned int rng_seed = 0;
    bool compare_direct_with_planned = false;
    bool count_pass_through_goals = false;
    int pass_through_candidate_limit = 0;

    // Intermediate cover graph, before terminal-to-terminal path construction.
    int num_cover_nodes = 0;
    int num_parameters = 2;

    int battery_capacity = 50000;
    double epsilon = 1.0;

    vector<int> cover_id;
    vector<deque<int>> cover_edge_nodes;
    vector<vector<int>> cover_edge_params;
    vector<map<int, vector<int>>> outgoing_cover_edges;
    vector<map<int, vector<int>>> incoming_cover_edges;

    // Indexed by cover_id; component 0 bounds the energy needed to reach the depot.
    vector<vector<int>> energy_to_depot;
    vector<int> max_parameter_values;

    string answer_string;

    // k counts road vertices, and the cover always retains depot/customers.
    void k_path_cover(int k);
    // Sparse paths stop at terminals unless expand_from_goals is enabled.
    map<int, vector<LabelPtr>> t_kpc_mls(int source, bool expand_from_goals);
    vector<vector<int>> backward_t_kpc_mls(int source);

    static bool check_domination(const vector<int>& v1, const vector<int>& v2) ;
    bool epsilon_check_domination(const vector<int>& v1, const vector<int>& v2) const ;
    static bool check_soc_domination(const vector<int>& v1, const vector<int>& v2) ;
    static bool check_soc_t_domination(const vector<int>& t, const vector<int>& v) ;
    static bool t_discard(int& t_set, vector<int>& new_vec);
    bool epsilon_t_discard(int& t_set, vector<int>& new_vec) const;
    static bool soc_t_discard(set<LabelPtr, soc_lex_compare>& closed_labels, set<vector<int>>& t_set, vector<int>& new_vec);
    static void update_t_set(int& t_set, int new_l) ;
    static void update_soc_t_set(set<vector<int>>& t_set, LabelPtr lab) ;

    vector<int> build_simple_timecons(int edge);
    vector<int> build_simple_soc(vector<int> edge_params) const;
    static vector<int> link_paths(vector<int>& current_timecons, vector<int>& suffix_timecons);
    static vector<int> link_soc_paths(vector<int>& current_soc, vector<int>& suffix_soc);

    static int add_energy(int& current_energy, int& added_energy);
    int mark_visited_goals_on_edge(int edge, vector<int>& goal_visited);
    bool validate_solution(const deque<int>& route_edges, const vector<int>& objectives,
                           int& route_count, string& reason);
    vector<pair<deque<int>, vector<int>>> bicriterion_ant_plan(int source, vector<int> source_parameters, vector<int>& goal_visited, bool allow_depot);

    pair<deque<int>, vector<int>> bicriterion_ant_jump_overhead(const vector<pair<deque<int>, vector<int>>>& paths_to_goals,
                                                                               vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                                                               double &lambda1, double &lambda2, mt19937 &rng);

    pair<deque<int>, vector<int>> bicriterion_svrp_ant_crawl(vector<vector<double>>& pheromones, double& alpha, double& beta, double& lambda1, double& lambda2, double& exploration_probability, double& evaporation_rate, double& initial_pheromone, mt19937& rng);
    int bicriterion_ant_pick_edge(vector<int>& available_edges, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, mt19937& rng);

    ShrinkGraph shrink_graph();

public:

    SVRPAntColony(Graph original_graph, int num_goals, int ants = 100, int iterations = 20,
                  int threads = 5, int battery = 50000, int cover_k = 24, unsigned int seed = 0,
                  bool compare_planned = false, bool count_pass_through = false,
                  int pass_through_candidates = 0, int vehicles = 0);

    string bicriterion_ant_svrp();
    SolverRunResult bicriterion_ant_svrp_result();
    // Legacy experiment entry point; the CLI uses the validated result method.
    string multicolony_ant_svrp();

};

#endif //ACO_VRP_ANTCOLONY_H
