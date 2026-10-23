#include <utility>
#include <vector>
#include <string>
#include <map>
#include <set>

using namespace std;

#ifndef ACO_VRP_GRAPH_H
#define ACO_VRP_GRAPH_H

// Road graph with internal IDs: depot 0, then customers, then transit vertices.
class Graph {
private:
    int num_nodes;
    int num_edges;
    int num_parameters = 2;

    vector<pair<int, int>> edge_points;
    vector<pair<int, int>> edge_parameters;

    vector<vector<int>> outgoing_edges;
    vector<vector<int>> incoming_edges;

    vector<int> restructure_ids; // Original ID -> internal ID.
    vector<int> id_to_goal;      // Internal terminal ID -> original ID.
    vector<int> original_ids;    // Internal ID -> original ID, including transit vertices.


public:
    Graph();
    Graph(int nnodes, vector<pair<int, int>> epoints, const vector<pair<int, int>>& eparameters, int& depot,  set<int>& goals);

    int get_num_nodes() const;
    int get_num_edges() const;
    int get_num_parameters() const;
    int get_id_to_goal(int id);
    pair<int, int> get_edge_parameters(int edge_index);
    pair<int, int> get_edge_points(int edge_index);
    vector<int> get_outgoing_edges(int vertex_index);
    vector<int> get_incoming_edges(int vertex_index);

    void augment_graph(set<int>& goals);
    int get_restructure_id(int node);
    int get_original_id(int internal_id);
};


#endif //ACO_VRP_GRAPH_H
