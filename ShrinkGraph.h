#include <vector>
#include <deque>
#include <map>

using namespace std;

#ifndef ACO_VRP_SHRINKGRAPH_H
#define ACO_VRP_SHRINKGRAPH_H


// Planning multigraph: parallel arcs retain different nondominated road paths.
class ShrinkGraph {
private:
    int num_nodes;
    int num_edges;
    int num_parameters = 2;

    vector<deque<int>> edge_nodes; // Full road paths; front/back are planning endpoints.
    vector<vector<int>> edge_parameters;

    map<int, vector<int>> outgoing_edges;
    map<int, vector<int>> incoming_edges;


public:
    ShrinkGraph();
    ShrinkGraph(int num_nodes, vector<deque<int>> shrink_edge_nodes, vector<vector<int>> shrink_edge_params);

    int get_num_nodes() const;
    int get_num_edges() const;
    int get_num_parameters() const;
    vector<int> get_edge_parameters(int edge_index);
    deque<int> get_edge_nodes(int edge_index);
    pair<int, int> get_edge_points(int edge_index);
    vector<int> get_outgoing_edges(int vertex_index);
    vector<int> get_incoming_edges(int vertex_index);
    vector<vector<long>> get_criteria_limits();
    vector<vector<vector<int>>> build_edge_matrix(vector<int>& shrinked_ids);

};


#endif //ACO_VRP_SHRINKGRAPH_H
