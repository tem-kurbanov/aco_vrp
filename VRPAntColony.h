#include <random>
#include "Graph.h"
#include "ShrinkGraph.h"
#include "Label.h"

#ifndef ACO_VRP_VRPANTCOLONY_H
#define ACO_VRP_VRPANTCOLONY_H



//struct lex_compare{
//    bool operator() (Label* lhs, Label* rhs) const {
//        for (int i = 0; i < (int)lhs->parameters.size(); ++i) {
//            if (lhs->parameters[i] < rhs->parameters[i])
//                return true;
//            else if (lhs->parameters[i] > rhs->parameters[i])
//                return false;
//        }
//
//        return lhs->node < rhs->node;
//    }
//};

class VRPAntColony {
private:
    // Variables:

    Graph graph;
    ShrinkGraph shrinked_graph;
    vector<int> shrinked_ids;
//    set<int> goals;
    int depot = 0;
    int num_criticals;

    vector<pair<int, int>> shrinked_edge_nodes;
    vector<vector<int>> shrinked_edge_parameters;

    const int K = 24;

    //kPC cover variables;
    int num_cover_nodes = 0;
    int num_cover_edges = 0;
    int num_parameters = 2;

    int battery_capacity = 50000;

    vector<int> cover_id;
    vector<deque<int>> cover_edge_nodes;
    vector<vector<int>> cover_edge_params;
    vector<map<int, vector<int>>> outgoing_cover_edges;
    vector<map<int, vector<int>>> incoming_cover_edges;

    vector<vector<int>> energy_to_depot;
    vector<int> max_parameter_values;

    int explorations = 0;
    int exploitations = 0;

    string answer_string;

    // Functions:

    void k_path_cover(int k);
    map<int, vector<Label*>> t_kpc_mls(int source);
    int closest_goal_search(int source, vector<int> source_params, set<int>& goal_set);

    int vrp_nearest_neighbor_heuristic(vector<int>& final_params);

    bool check_domination(const vector<int>& v1, const vector<int>& v2) const;
    bool check_t_domination(const vector<int>& t, const vector<int>& v) const;
    bool check_strict_domination(const vector<int>& v1, const vector<int>& v2) const;
    bool t_discard(set<Label*, lex_compare>& closed_labels, set<vector<int>>& t_set, vector<int>& new_vec);
    void update_t_set(set<vector<int>>& t_set, Label* lab) const;

    vector<int> build_simple_timecons(int edge);
    vector<int> link_paths(vector<int>& current_timecons, vector<int>& suffix_timecons);

    int add_energy(int& current_energy, int& added_energy);

    void print_out_route_coords(deque<int> &route);

    void bicriterion_two_opt(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                           vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                           double &lambda1, double &lambda2, random_device &rd);

    void bicriterion_destination_exchange(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                            vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                            double &lambda1, double &lambda2, random_device &rd);

    ShrinkGraph shrink_graph();

    void save_route(deque<int> &route_edges);

    pair<deque<int>, vector<int>> bicriterion_vrp_ant_crawl(vector<vector<double>>& pheromones, double& alpha, double& beta, double& lambda1, double& lambda2, double& exploration_probability, double& evaporation_rate, double& initial_pheromone);
    int bicriterion_ant_pick_edge(vector<int>& available_edges, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, random_device& rd);

public:

    VRPAntColony(Graph original_graph, int num_goals);

    string bicriterion_ant_vrp();

};

#endif //ACO_VRP_VRPANTCOLONY_H
