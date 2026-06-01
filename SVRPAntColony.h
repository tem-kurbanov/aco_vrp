#include <random>
#include "Graph.h"
#include "ShrinkGraph.h"
#include "Label.h"

#ifndef ACO_VRP_ANTCOLONY_H
#define ACO_VRP_ANTCOLONY_H

//struct Label{
//    int node;
//    vector<int> parameters;
//    Label *parent;
//    int used_edge;
//
//    Label(int n, vector<int> params, Label *p, int edge) : node(n), parameters(move(params)), parent(p), used_edge(edge) {}
//
//};

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

//struct soc_lex_compare{
//    bool operator() (Label* lhs, Label* rhs) const {
//        for (int i = 0; i < (int)lhs->parameters.size() - 1; ++i) {
//            if (lhs->parameters[i] < rhs->parameters[i])
//                return true;
//            else if (lhs->parameters[i] > rhs->parameters[i])
//                return false;
//        }
//
//        if (lhs->parameters[(int)lhs->parameters.size() - 1] > rhs->parameters[(int)lhs->parameters.size() - 1])
//            return true;
//        else if (lhs->parameters[(int)lhs->parameters.size() - 1] < rhs->parameters[(int)lhs->parameters.size() - 1])
//            return false;
//
//        return lhs->node < rhs->node;
//    }
//};

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

    const int K = 24;

    //kPC cover variables;
    int num_cover_nodes = 0;
    int num_cover_edges = 0;
    int num_parameters = 2;
    int goal_border = 0;

    int battery_capacity = 50000;
    double epsilon = 0.98;

    int num_steps = 0;
    int num_plans = 0;

    vector<int> cover_id;
    vector<deque<int>> cover_edge_nodes;
    vector<vector<int>> cover_edge_params;
    vector<map<int, vector<int>>> outgoing_cover_edges;
    vector<map<int, vector<int>>> incoming_cover_edges;

    vector<set<int>> in_connections;
    vector<set<int>> out_connections;

    vector<vector<int>> energy_to_depot;
    vector<int> max_parameter_values;

    int explorations = 0;
    int exploitations = 0;

    double gamma = 1.0;

    string answer_string;

    // Functions:
    void k_path_cover(int k);
    map<int, vector<Label*>> t_kpc_mls(int source, bool expand_from_goals);
    vector<vector<int>> backward_t_kpc_mls(int source);
    int closest_goal_search(int source, vector<int> source_params, vector<bool>& goal_visited, deque<int> &route_nodes, vector<int> &route_params);

    int svrp_nearest_neighbor_heuristic(vector<int>& final_params);

    bool invert_route(vector<pair<int, vector<int>>>& possible_edges, deque<int> &initial_nodes, deque<int> &initial_edges, vector<int> &initial_parameters,
                      vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                      double &lambda1, double &lambda2, random_device &rd);

    static bool check_domination(const vector<int>& v1, const vector<int>& v2) ;
    bool epsilon_check_domination(const vector<int>& v1, const vector<int>& v2) const ;
    static bool check_strict_domination(const vector<int>& v1, const vector<int>& v2) ;
    static bool check_soc_domination(const vector<int>& v1, const vector<int>& v2) ;
    bool check_t_domination(const vector<int>& t, const vector<int>& v) const;
    static bool check_soc_t_domination(const vector<int>& t, const vector<int>& v) ;
    static bool t_discard(int& t_set, vector<int>& new_vec);
    bool epsilon_t_discard(int& t_set, vector<int>& new_vec) const;
    static bool soc_t_discard(set<Label*, soc_lex_compare>& closed_labels, set<vector<int>>& t_set, vector<int>& new_vec);
    static void update_t_set(int& t_set, int new_l) ;
    static void update_soc_t_set(set<vector<int>>& t_set, Label* lab) ;

    vector<int> build_simple_timecons(int edge);
    vector<int> build_simple_soc(vector<int> edge_params) const;
    static vector<int> link_paths(vector<int>& current_timecons, vector<int>& suffix_timecons);
    static vector<int> link_soc_paths(vector<int>& current_soc, vector<int>& suffix_soc);

    static int add_energy(int& current_energy, int& added_energy);
    int bicriterion_ant_pick_overhead(vector<pair<deque<int>, vector<int>>>& planned_routes, vector<vector<double>>& pheromones, double& exploitation_probability, double& alpha, double& beta, double& lambda1, double& lambda2, random_device& rd);
    vector<pair<deque<int>, vector<int>>> bicriterion_ant_plan(int source, vector<int> source_parameters, vector<int>& goal_visited);

    void bicriterion_two_opt(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                             vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                             double &lambda1, double &lambda2, random_device &rd);

    void bicriterion_destination_exchange(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                          vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                          double &lambda1, double &lambda2, random_device &rd);

    pair<deque<int>, vector<int>> bicriterion_ant_jump_overhead(const vector<pair<deque<int>, vector<int>>>& paths_to_goals,
                                                                               vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                                                               double &lambda1, double &lambda2, random_device &rd);

    void vertex_reinsertion(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                           vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                           double &lambda1, double &lambda2, random_device &rd);

    bool invert_subtour(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters, int dead_end, const vector<pair<deque<int>, vector<int>>>& paths_to_goals,
                                       vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                       double &lambda1, double &lambda2, random_device &rd);

    vector<pair<deque<int>, vector<int>>> plan_to_goal(int source, int goal, const vector<int>& source_parameters, set<int>& available_vertices);


    void print_out_route_coords(deque<int> &route);

    pair<deque<int>, vector<int>> bicriterion_svrp_ant_crawl(vector<vector<double>>& pheromones, double& alpha, double& beta, double& lambda1, double& lambda2, double& exploration_probability, double& evaporation_rate, double& initial_pheromone);
    int bicriterion_ant_pick_edge(vector<int>& available_edges, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, random_device& rd);


    ShrinkGraph shrink_graph();

    void save_route(deque<int> &route_edges);

public:

    SVRPAntColony(Graph original_graph, int num_goals);

    string bicriterion_ant_svrp();
    string multicolony_ant_svrp();

};


#endif //ACO_VRP_ANTCOLONY_H
