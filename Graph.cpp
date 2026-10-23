#include "Graph.h"
#include <fstream>
#include <iostream>
#include <map>
#include <utility>
#include <set>

using namespace std;

Graph::Graph(int nnodes, vector<pair<int, int>> epoints, const vector<pair<int, int>>& eparameters, int& depot,  set<int>& goals) {

    num_nodes = nnodes;
    num_edges = epoints.size();

    restructure_ids.assign(num_nodes, -1);
    original_ids.assign(num_nodes, -1);
    id_to_goal.assign(goals.size() + 1, -1);
    restructure_ids[depot] = 0;
    original_ids[0] = depot;
    id_to_goal[0] = depot;
    int counter = 1;
    for (int i = 0; i < num_nodes; ++i) {
        if (goals.find(i) != goals.end()) {
            restructure_ids[i] = counter;
            original_ids[counter] = i;
            id_to_goal[counter] = i;
            counter++;
        }
    }
    for (int i = 0; i < num_nodes; ++i) {
        if (restructure_ids[i] == -1) {
            restructure_ids[i] = counter;
            original_ids[counter] = i;
            counter++;
        }
    }

    outgoing_edges.resize(num_nodes);
    incoming_edges.resize(num_nodes);

    int index = 0;
    for (int e = 0; e < num_edges; ++e) {
        int start = epoints[e].first;

        int end = epoints[e].second;

        int p1 = eparameters[e].first;
        int p2 = eparameters[e].second;

        edge_points.emplace_back(restructure_ids[start], restructure_ids[end]);
        edge_parameters.emplace_back(p1, p2);

        outgoing_edges[restructure_ids[start]].push_back(index);
        incoming_edges[restructure_ids[end]].push_back(index);

        index += 1;
    }
}

int Graph::get_num_edges() const {
    return num_edges;
}

int Graph::get_num_nodes() const {
    return num_nodes;
}

int Graph::get_num_parameters() const {
    return num_parameters;
}

pair<int, int> Graph::get_edge_parameters(int edge_index) {
    return edge_parameters[edge_index];
}

pair<int, int> Graph::get_edge_points(int edge_index) {
    return edge_points[edge_index];
}

vector<int> Graph::get_outgoing_edges(int vertex_index) {
    return outgoing_edges[vertex_index];
}

vector<int> Graph::get_incoming_edges(int vertex_index) {
    return incoming_edges[vertex_index];
}

int Graph:: get_id_to_goal(int id) {
    return id_to_goal[id];
}

Graph::Graph() {
    num_nodes = 0;
    num_edges = 0;
}

void Graph::augment_graph(set<int>& goals) {
    for (int g: goals) {
        int new_id = num_nodes;

        for (int out_edge: outgoing_edges[g]) {
            edge_points[out_edge].first = new_id;
        }
        for (int in_edge: incoming_edges[g]) {
            edge_points[in_edge].second = new_id;
        }

        vector<int> new_out = outgoing_edges[g];
        vector<int> new_in = incoming_edges[g];

        outgoing_edges.push_back(new_out);
        incoming_edges.push_back(new_in);

        outgoing_edges[g] = vector<int>({num_edges});
        incoming_edges[g] = vector<int>({num_edges + 1});

        outgoing_edges[new_id].push_back(num_edges + 1);
        incoming_edges[new_id].push_back(num_edges);

        edge_points.emplace_back(g, new_id);
        edge_points.emplace_back(new_id, g);

        edge_parameters.emplace_back(num_parameters, 1);
        edge_parameters.emplace_back(num_parameters, 1);

        num_nodes += 1;
        num_edges += 2;

        restructure_ids[new_id] = g;
    }


}

int Graph::get_restructure_id(int node) {
        return restructure_ids[node];
}

int Graph::get_original_id(int internal_id) {
    if (internal_id >= 0 && internal_id < num_nodes) {
        return original_ids[internal_id];
    }
    return -1;
}


