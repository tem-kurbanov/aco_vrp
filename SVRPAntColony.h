#include <random>
#include "Graph.h"
#include "ShrinkGraph.h"
#include "Label.h"

#ifndef ACO_VRP_ANTCOLONY_H
#define ACO_VRP_ANTCOLONY_H

class SVRPAntColony {
private:
    // Variables:
    Graph graph;
    ShrinkGraph shrinked_graph;
    int num_criticals;
    int depot = 0;
    int num_agents;

    vector<pair<int, int>> shrinked_edge_nodes;
    vector<vector<int>> shrinked_edge_parameters;

    int k_cover = 24;
    int num_ants = 100;
    int iteration_limit = 20;
    int thread_count = 5;
    unsigned int rng_seed = 0;

    //kPC cover variables;
    int num_cover_nodes = 0;
    int num_cover_edges = 0;
    int num_parameters = 2;
    int goal_border = 0;

    int battery_capacity = 50000;
    double epsilon = 0.98;

    vector<int> cover_id;
    vector<deque<int>> cover_edge_nodes;
    vector<vector<int>> cover_edge_params;
    vector<map<int, vector<int>>> outgoing_cover_edges;
    vector<map<int, vector<int>>> incoming_cover_edges;

    vector<set<int>> in_connections;
    vector<set<int>> out_connections;

    vector<vector<int>> energy_to_depot;
    vector<int> max_parameter_values;

    double gamma = 1.0;

    string answer_string;

    // Functions:
    void k_path_cover(int k);
    map<int, vector<LabelPtr>> t_kpc_mls(int source, bool expand_from_goals);
    vector<vector<int>> backward_t_kpc_mls(int source);
    int closest_goal_search(int source, vector<int> source_params, vector<bool>& goal_visited, deque<int> &route_nodes, vector<int> &route_params);

    int svrp_nearest_neighbor_heuristic(vector<int>& final_params);

    bool invert_route(vector<pair<int, vector<int>>>& possible_edges, deque<int> &initial_nodes, deque<int> &initial_edges, vector<int> &initial_parameters,
                      vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                      double &lambda1, double &lambda2, mt19937 &rng);

    static bool check_domination(const vector<int>& v1, const vector<int>& v2) ;
    bool epsilon_check_domination(const vector<int>& v1, const vector<int>& v2) const ;
    static bool check_strict_domination(const vector<int>& v1, const vector<int>& v2) ;
    static bool check_soc_domination(const vector<int>& v1, const vector<int>& v2) ;
    bool check_t_domination(const vector<int>& t, const vector<int>& v) const;
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
    int bicriterion_ant_pick_overhead(vector<pair<deque<int>, vector<int>>>& planned_routes, vector<vector<double>>& pheromones, double& exploitation_probability, double& alpha, double& beta, double& lambda1, double& lambda2, mt19937& rng);
    vector<pair<deque<int>, vector<int>>> bicriterion_ant_plan(int source, vector<int> source_parameters, vector<int>& goal_visited);

    void bicriterion_two_opt(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                             vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                             double &lambda1, double &lambda2, mt19937 &rng);

    void bicriterion_destination_exchange(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                          vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                          double &lambda1, double &lambda2, mt19937 &rng);

    pair<deque<int>, vector<int>> bicriterion_ant_jump_overhead(const vector<pair<deque<int>, vector<int>>>& paths_to_goals,
                                                                               vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                                                               double &lambda1, double &lambda2, mt19937 &rng);

    void vertex_reinsertion(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                           vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                           double &lambda1, double &lambda2, mt19937 &rng);

    bool invert_subtour(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters, int dead_end, const vector<pair<deque<int>, vector<int>>>& paths_to_goals,
                                       vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                       double &lambda1, double &lambda2, mt19937 &rng);

    vector<pair<deque<int>, vector<int>>> plan_to_goal(int source, int goal, const vector<int>& source_parameters, set<int>& available_vertices);


    void print_out_route_coords(deque<int> &route);

    pair<deque<int>, vector<int>> bicriterion_svrp_ant_crawl(vector<vector<double>>& pheromones, double& alpha, double& beta, double& lambda1, double& lambda2, double& exploration_probability, double& evaporation_rate, double& initial_pheromone, mt19937& rng);
    int bicriterion_ant_pick_edge(vector<int>& available_edges, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, mt19937& rng);


    ShrinkGraph shrink_graph();

    void save_route(deque<int> &route_edges);

public:

    SVRPAntColony(Graph original_graph, int num_goals, int ants = 100, int iterations = 20,
                  int threads = 5, int battery = 50000, int cover_k = 24, unsigned int seed = 0);

    string bicriterion_ant_svrp();
    string multicolony_ant_svrp();

};


#endif //ACO_VRP_ANTCOLONY_H
