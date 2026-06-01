//
// Created by Tim on 04.04.2022.
//

#include "ShrinkGraph.h"

#include <utility>


using namespace std;

ShrinkGraph::ShrinkGraph() {
    num_nodes = 0;
    num_edges = 0;
}

ShrinkGraph::ShrinkGraph(int num_nodes, vector<deque<int>> shrink_edge_nodes, vector<vector<int>> shrink_edge_params) {
    this->num_nodes = num_nodes;
    edge_nodes = move(shrink_edge_nodes);
    edge_parameters = move(shrink_edge_params);
    num_edges = (int)edge_nodes.size();

    for (int e = 0; e < num_edges; ++e) {
        int start = edge_nodes[e].front();
        int end = edge_nodes[e].back();

        outgoing_edges[start].push_back(e);
        incoming_edges[end].push_back(e);
    }
}

int ShrinkGraph::get_num_nodes() const {
    return num_nodes;
}

int ShrinkGraph::get_num_edges() const {
    return num_edges;
}

int ShrinkGraph::get_num_parameters() const {
    return num_parameters;
}

vector<int> ShrinkGraph::get_edge_parameters(int edge_index) {
    return edge_parameters[edge_index];
}

deque<int> ShrinkGraph::get_edge_nodes(int edge_index) {
    return edge_nodes[edge_index];
}

vector<int> ShrinkGraph::get_outgoing_edges(int vertex_index) {
    return outgoing_edges[vertex_index];
}

vector<int> ShrinkGraph::get_incoming_edges(int vertex_index) {
    return incoming_edges[vertex_index];
}

vector<vector<long>> ShrinkGraph::get_criteria_limits() {
    vector<vector<long>> limits(num_parameters, vector<long>(2, 0));
    for (auto e: edge_parameters) {
        for (int i = 0; i < num_parameters; ++i) {
            limits[i][1] += e[i];
        }
    }
    return limits;

}

vector<vector<vector<int>>> ShrinkGraph::build_edge_matrix(vector<int>& shrinked_ids) {
map<int,int> backward_ids;
for (int i = 0; i < shrinked_ids.size(); ++i) {
backward_ids[shrinked_ids[i]] = i;
}
vector<vector<vector<int>>> edge_matrix(num_nodes, vector<vector<int>>(num_nodes));
for (int e = 0; e < num_edges; ++e) {
int start = edge_nodes[e].front();
int end = edge_nodes[e].back();
edge_matrix[backward_ids[start]][backward_ids[end]].push_back(e);
}

return edge_matrix;
}

pair<int, int> ShrinkGraph::get_edge_points(int edge_index) {
    return make_pair(edge_nodes[edge_index].front(), edge_nodes[edge_index].back());
}
