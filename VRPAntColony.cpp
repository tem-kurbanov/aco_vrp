#include "VRPAntColony.h"
#include "ShrinkGraph.h"
#include <chrono>
#include <iostream>
#include <stack>
#include <utility>
#include <fstream>
#include <random>
#include <climits>
#include <algorithm>
#include <cmath>
#include <omp.h>

VRPAntColony::VRPAntColony(Graph original_graph, int num_goals, int ants, int iterations,
                           int threads, int battery, int cover_k, unsigned int seed) {
    this->graph = std::move(original_graph);
    this->depot = 0;
    this->num_criticals = num_goals + 1;
    this->num_ants = ants;
    this->iteration_limit = iterations;
    this->thread_count = threads;
    this->battery_capacity = battery;
    this->k_cover = cover_k;
    this->rng_seed = seed;
}


void VRPAntColony::k_path_cover(int k) {
    auto t1 = chrono::high_resolution_clock::now();

    int num_nodes = graph.get_num_nodes();

//    cout << "Starting " << k << "-PC construction" << endl;
    cover_id.assign(num_nodes, -1);
    vector<bool> in_cover(num_nodes, true);
    num_cover_nodes = 0;

    // Pick KPC nodes
    for (int n = 0; n < num_nodes; ++n) {
        // Check if the node id is present in the original graph
//        if (n % 10000 == 0)
//            cout << n << endl;

        // Docks stay in the cover graph
        if (n < num_criticals){
            cover_id[n] = num_cover_nodes;
            num_cover_nodes += 1;
            continue;
        }

        bool stays = false;

        // Forward DFS
        set<deque<int>> built_paths;

        stack<pair<int, deque<int>>> to_expand;
        to_expand.emplace(n, deque<int>({n}));

        while (!to_expand.empty()) {
            pair<int, deque<int>> current_pair = to_expand.top();
            int current_node = current_pair.first;
            deque<int> current_path = current_pair.second;
            to_expand.pop();

            if (current_path.size() + 1 == k) {
                stays = true;
                break;
            }

            for (int next_edge : graph.get_outgoing_edges(current_node)) {

                int next_node = graph.get_edge_points(next_edge).second;

                if (find(current_path.begin(), current_path.end(), next_node) != current_path.end())
                    continue;
                if (in_cover[next_node]) {
                    built_paths.insert(current_path);
                    continue;
                }

                deque<int> new_path(current_path);
                new_path.push_back(next_node);
                to_expand.emplace(next_node, new_path);
            }
        }

        if (stays) {
            cover_id[n] = num_cover_nodes;
            num_cover_nodes += 1;
            continue;
        }
        // Backward DFS
        for (const deque<int>& init_path : built_paths) {
            stack<pair<int, deque<int>>> expand;
            expand.emplace(n, init_path);
            while (!expand.empty()) {
                pair<int, deque<int>> current_pair = expand.top();
                int current_node = current_pair.first;
                deque<int> current_path = current_pair.second;
                expand.pop();

                if (current_path.size() + 1 == k) {
                    stays = true;
                    break;
                }

                for (int next_edge : graph.get_incoming_edges(current_node)) {

                    int next_node = graph.get_edge_points(next_edge).first;
                    if (in_cover[next_node] || find(current_path.begin(), current_path.end(), next_node) != current_path.end()) {
                        continue;
                    }

                    deque<int> new_path(current_path);
                    new_path.push_front(next_node);
                    expand.emplace(next_node, new_path);
                }
            }
        }

        if (!stays) {
            in_cover[n] = false;
        }
        else {
            cover_id[n] = num_cover_nodes;
            num_cover_nodes += 1;
        }
    }

//    cout << "Initial number of vertices -- " << num_nodes << endl;
//    cout << "Number of cover vertices -- " << num_cover_nodes << endl;
//    cout << "Relative vertex percentage -- " << ((float)num_cover_nodes / (float)num_nodes) * 100 << endl;

    outgoing_cover_edges.assign(num_cover_nodes, map<int, vector<int>>());
    incoming_cover_edges.assign(num_cover_nodes, map<int, vector<int>>());

    unsigned long num_init_cover_edges = 0;

    unsigned long dom_pruned = 0;

    // Build cover edges
    for (int n = 0; n < num_nodes; ++n) {
        if (!in_cover[n])
            continue;

        map<int, vector<deque<int>>> cover_nodes;
        map<int, vector<vector<int>>> cover_params;

        stack<pair<deque<int>, vector<int>>> to_expand;
        to_expand.emplace(deque<int>({n}), vector<int>({0, 0}));
        while (!to_expand.empty()) {
            deque<int> current_path = to_expand.top().first;
            vector<int> current_criteria = to_expand.top().second;
            to_expand.pop();

            int current_node = current_path.back();
            for (int next_edge : graph.get_outgoing_edges(current_node)) {
                vector<int> suffix = build_simple_timecons(next_edge);
                vector<int> new_criteria = link_paths(current_criteria, suffix);

                int next_node = graph.get_edge_points(next_edge).second;
                if (find(current_path.begin(), current_path.end(), next_node) != current_path.end())
                    continue;

                // Check if cover node is met
                if (in_cover[next_node]) {
                    deque<int> full_path(current_path);
                    full_path.push_back(next_node);

                    // Build the new cover edge and add it to the graph
                    if (cover_nodes.find(next_node) == cover_nodes.end()) {
                        cover_nodes[next_node] = vector<deque<int>>();
                        cover_params[next_node] = vector<vector<int>>();
                    }

                    num_init_cover_edges += 1;

                    // Domination pruning
                    bool is_dominated = false;
                    deque<int> to_remove;
                    for (int v = 0; v < cover_params[next_node].size(); ++v) {
                        auto& vec = cover_params[next_node][v];
                        if (check_domination(vec, new_criteria)) {
                            is_dominated = true;

                        }
                        else if (check_domination(new_criteria, vec)) {
                            to_remove.push_front(v);
                        }
                    }
                    if (is_dominated) {
                        dom_pruned += 1;
                        continue;
                    }

                    while (!to_remove.empty()) {
                        dom_pruned += 1;
                        num_cover_edges -= 1;
                        int ind = to_remove.front();
                        to_remove.pop_front();
                        cover_params[next_node].erase(cover_params[next_node].begin() + ind);
                        cover_nodes[next_node].erase(cover_nodes[next_node].begin() + ind);
                    }

                    num_cover_edges += 1;
                    cover_nodes[next_node].push_back(full_path);
                    cover_params[next_node].push_back(new_criteria);

                    continue;
                }

                deque<int> new_path(current_path);
                new_path.push_back(next_node);
                to_expand.push(make_pair(new_path, new_criteria));
            }
        }

        for (auto &it: cover_nodes) {
            for (auto &d: it.second) {
                cover_edge_nodes.push_back(d);
                outgoing_cover_edges[cover_id[n]][it.first].push_back((int)cover_edge_nodes.size() - 1);
                incoming_cover_edges[cover_id[it.first]][n].push_back((int)cover_edge_nodes.size() - 1);
            }
        }

        for (auto &it: cover_params) {
            for (auto &d: it.second) {
                cover_edge_params.push_back(d);
            }
        }

//        outgoing_cover_edge_nodes[cover_id[n]] = cover_nodes;
//        outgoing_cover_edge_params[cover_id[n]] = cover_params;

    }


//    cout << "Initial number of edges -- " << graph.get_num_edges() << endl;
//    cout << "Number of cover edges -- " << num_init_cover_edges << endl;

    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;
//    cout << "Construction duration: " << duration << endl;
//
//    cout << "Number of cover edges after pruning -- " << num_cover_edges << endl;
//    cout << "% pruned edges -- " << ((float)(num_init_cover_edges - num_cover_edges) / (float)num_init_cover_edges) * 100 << endl;
//    cout << "Pruned by domination -- " << dom_pruned << endl;
//
//    cout << "Ended " << k << "-PC construction" << endl;
//    cout << endl;

}


map<int, vector<LabelPtr>> VRPAntColony::t_kpc_mls(int source){
    auto t1 = chrono::high_resolution_clock::now();

    vector<set<LabelPtr, lex_compare>> closed_labels(num_cover_nodes);
    vector<set<LabelPtr, lex_compare>> open_labels(num_cover_nodes);
    vector<set<vector<int>>> t_sets(num_cover_nodes);
    set<LabelPtr, lex_compare> to_expand;
    long num_labels = 0;
    auto source_label = make_shared<Label>(source, vector<int>(num_parameters, 0), nullptr, -1);
    num_labels++;

    to_expand.insert(source_label);
    open_labels[cover_id[source]].insert(source_label);

    int num_iters = 0;

    while (!to_expand.empty()){
        LabelPtr current_label = *to_expand.begin();
        to_expand.erase(to_expand.begin());

        num_iters += 1;

        int current_node = current_label->node;
        vector<int> current_params = current_label->parameters;

        open_labels[cover_id[current_node]].erase(current_label);
        closed_labels[cover_id[current_node]].insert(current_label);
        update_t_set(t_sets[cover_id[current_node]], current_label);

        for (auto& p : outgoing_cover_edges[cover_id[current_node]]){
            int next_node = p.first;

            if (current_label->parent && next_node == current_label->parent->node)
                continue;
            vector<int> param_vectors = p.second;       // Parameter vectors of possible routes to next_node
            for (int i = 0; i < param_vectors.size(); ++i){          // Every parameter vector produces a new label
                vector<int>& params = cover_edge_params[param_vectors[i]];
                vector<int> new_criteria = link_paths(current_params, params);

                // Check for t-dominance by closed labels
                bool t_discarded = t_discard(closed_labels[cover_id[next_node]], t_sets[cover_id[next_node]], new_criteria);
                if (t_discarded)
                    continue;

                // Check if is dominated by or dominates open labels
                set<LabelPtr, lex_compare>& other_labels = open_labels[cover_id[next_node]];
                bool is_dominated = false;
                stack<LabelPtr> to_remove;
                for (LabelPtr l : other_labels){
                    if (check_domination(l->parameters, new_criteria)){
                        is_dominated = true;
                        break;
                    }
                    else if (check_domination(new_criteria, l->parameters)){
                        to_remove.push(l);
                    }
                }

                if (is_dominated)
                    continue;

                while (!to_remove.empty()){
                    auto rit = to_remove.top();
                    to_remove.pop();
                    to_expand.erase(rit);
                    other_labels.erase(rit);
                    num_labels--;
                }


                auto new_label = make_shared<Label>(next_node, new_criteria, current_label, i);
                num_labels++;

                open_labels[cover_id[next_node]].insert(new_label);
                to_expand.insert(new_label);
            }
        }
    }

    map<int, vector<LabelPtr>> bag;

    for (int n = 0; n < graph.get_num_nodes(); ++n) {
        if (cover_id[n] >= 0 && !closed_labels[cover_id[n]].empty()) {
            bag[n] = vector<LabelPtr>(closed_labels[cover_id[n]].begin(), closed_labels[cover_id[n]].end());
        }
    }

    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;
//    cout << "Query duration: " << duration << endl;

    return bag;
}


int VRPAntColony::closest_goal_search(int source, vector<int> source_params, set<int>& goal_set) {
//    auto t1 = chrono::high_resolution_clock::now();

    int num_edges = shrinked_graph.get_num_edges();

    int chosen_edge = -1;
    int next_vertex = -1;
    int min_dist = INT_MAX;

    for (int g : goal_set) {
        vector<int> possible_edges = shrinked_graph.get_outgoing_edges(source);
        for (int e: possible_edges) {
            if (shrinked_graph.get_edge_nodes(e).back() == g) {
                int new_energy = add_energy(source_params[1], shrinked_graph.get_edge_parameters(e)[1]);
                int edge_to_depot = -1;
                for (int ne: shrinked_graph.get_outgoing_edges(g)) {
                    if (shrinked_graph.get_edge_nodes(ne).back() == depot) {
                        edge_to_depot = ne;
                        break;
                    }
                }
                if (battery_capacity - add_energy(new_energy, shrinked_graph.get_edge_parameters(edge_to_depot)[1]) >= 0){
                    if (shrinked_graph.get_edge_parameters(e)[0] < min_dist) {
                        min_dist = shrinked_graph.get_edge_parameters(e)[0];
                        chosen_edge = e;
                    }
                }
            }
        }
    }

    // If no reachable goal is found, go to depot
    if (chosen_edge == -1) {
        for (int e: shrinked_graph.get_outgoing_edges(source)) {
            if (shrinked_graph.get_edge_nodes(e).back() == depot) {
                vector<int> params = shrinked_graph.get_edge_parameters(e);
                if (battery_capacity - add_energy(source_params[1], params[1]) >= 0 && min_dist > params[0]) {
                    chosen_edge = e;
                    min_dist = params[0];
                }
            }
        }
    }

    return chosen_edge;
}


bool VRPAntColony::check_domination(const vector<int>& v1, const vector<int>& v2) const{
    for (int i = 0; i < v1.size(); ++i) {
        if (v1[i] > v2[i])
            return false;
    }
    return true;
}

bool VRPAntColony::check_strict_domination(const vector<int>& v1, const vector<int>& v2) const{
    if ((v1[0] < v2[0]) && (v1[1] <= v2[1])) {
        return true;
    }
    if ((v1[0] <= v2[0]) && (v1[1] < v2[1])) {
        return true;
    }
    return false;
}


bool VRPAntColony::check_t_domination(const vector<int>& t, const vector<int>& v) const{
    for (int i = 0; i < t.size(); ++i) {
        if (t[i] > v[i + 1])
            return false;
    }
    return true;
}


bool VRPAntColony::t_discard(set<LabelPtr, lex_compare>& closed_labels, set<vector<int>>& t_set, vector<int>& new_vec){
    // Check first parameter dominance
    if (closed_labels.empty())
        return false;
    auto& max_p = *(closed_labels.rbegin());

    if (new_vec[0] < max_p->parameters[0])
        return false;

    // Check dominance of t-set
    for (auto& lab : t_set){
        if (check_t_domination(lab, new_vec)){
            return true;
        }
    }

    return false;
}


void VRPAntColony::update_t_set(set<vector<int>>& t_set, LabelPtr lab) const{
    vector<int> new_vec(lab->parameters.begin() + 1, lab->parameters.end());
    for (const vector<int>& v : t_set){
        if (check_domination(v, new_vec))
            return;
    }

    auto inserted = t_set.insert(new_vec);
    auto it = inserted.first;
    if (inserted.second){
        it++;
        vector<set<vector<int>>::iterator> to_remove;
        while(it != t_set.end()){
            if (check_domination(new_vec, (*it))){
                to_remove.push_back(it);
            }
            it++;
        }

        for (auto i = to_remove.rbegin(); i != to_remove.rend(); ++i){
            t_set.erase(*i);
        }
    }
}


vector<int> VRPAntColony::build_simple_timecons(int edge){
    int cons = graph.get_edge_parameters(edge).second;
    vector<int> new_timesoc({
                                    graph.get_edge_parameters(edge).first,                               // time
                                    cons});    // consumption

    return new_timesoc;
}


vector<int> VRPAntColony::link_paths(vector<int>& current_timecons, vector<int>& suffix_timecons){
    vector<int> full_timesoc(num_parameters, 0);
    full_timesoc[0] = current_timecons[0] + suffix_timecons[0];
    int cons = current_timecons[1] + suffix_timecons[1];
    if (cons > battery_capacity) {
        cons = battery_capacity + 1;
    } else if (cons < 0) {
        cons = 0;
    }
    full_timesoc[1] = cons;
    return full_timesoc;
}


ShrinkGraph VRPAntColony::shrink_graph() {
    if (cover_edge_params.empty())
        k_path_cover(k_cover);


    vector<deque<int>> shrink_edge_nodes;
    vector<vector<int>> shrink_edge_params;
    vector<vector<deque<int>>> shrink_edge_nodes_by_source(num_criticals);
    vector<vector<vector<int>>> shrink_edge_params_by_source(num_criticals);

#pragma omp parallel for num_threads(thread_count)
    for (int s = 0; s < num_criticals; ++s) {
        map<int, vector<LabelPtr>> paths = t_kpc_mls(s);

        // Turn produced paths to other goals into edges
        for (int goal = 0; goal < num_criticals; ++goal) {
            if (goal == s || paths.find(goal) == paths.end()) {
                continue;
            }

            vector<vector<int>> edge_params;
            vector<deque<int>> edge_nodes;

            for (LabelPtr start: paths[goal]) {
                vector<int> current_params;
                deque<int> current_nodes;

                LabelPtr current_label = start;
                // Add nodes of the current label to the full path
                while (current_label->node != s) {
                    int end_node = current_label->node;
                    int start_node = current_label->parent->node;
                    deque<int> traversed_nodes = cover_edge_nodes[outgoing_cover_edges[cover_id[start_node]][end_node][current_label->used_edge]];
                    for (auto it = traversed_nodes.rbegin(); it != traversed_nodes.rend() - 1; ++it) {
                        current_nodes.push_front(*it);
                    }
                    current_label = current_label->parent;
                }
                current_nodes.push_front(s);

                current_params = start->parameters;

                shrink_edge_params_by_source[s].push_back(current_params);
                shrink_edge_nodes_by_source[s].push_back(current_nodes);
            }
        }

    }

    for (int s = 0; s < num_criticals; ++s) {
        for (int i = 0; i < shrink_edge_nodes_by_source[s].size(); ++i) {
            shrink_edge_params.push_back(shrink_edge_params_by_source[s][i]);
            shrink_edge_nodes.push_back(shrink_edge_nodes_by_source[s][i]);
        }
    }

    ShrinkGraph shrink_graph(num_criticals, shrink_edge_nodes, shrink_edge_params);

    return shrink_graph;
}

// Main solution function for VRP
string VRPAntColony::bicriterion_ant_vrp() {
    // hyperparameters
    double evaporation_rate = 0.05;
    double exploration_probability = 0.1;
    double alpha = 1;
    double beta = 3;

    // Shrink graph
    auto s1 = chrono::high_resolution_clock::now();
    shrinked_graph = shrink_graph();
    auto s2 = chrono::high_resolution_clock::now();
    double s_dur = chrono::duration_cast<chrono::nanoseconds>( s2 - s1 ).count();
    s_dur *= 1e-9;
    answer_string += std::to_string(shrinked_graph.get_num_nodes()) + "," + std::to_string(shrinked_graph.get_num_edges()) + "," + to_string(s_dur) + " ";

    shrinked_edge_nodes.resize(shrinked_graph.get_num_edges());
    shrinked_edge_parameters.resize(shrinked_graph.get_num_edges());
    for (int i = 0; i < shrinked_graph.get_num_edges(); ++i) {
        shrinked_edge_nodes[i] = make_pair(shrinked_graph.get_edge_nodes(i).front(), shrinked_graph.get_edge_nodes(i).back());
        shrinked_edge_parameters[i] = shrinked_graph.get_edge_parameters(i);
    }

//    cout << "Shrinked graph size: " << shrinked_edge_nodes.size() << endl;

    for (int g = 0; g < num_criticals; ++g) {
        if (shrinked_graph.get_outgoing_edges(g).empty() || shrinked_graph.get_incoming_edges(g).empty()) {
//            cout << "Vertex is not connected" << endl;
            return {};
        }
    }
    if (shrinked_graph.get_outgoing_edges(depot).empty() || shrinked_graph.get_incoming_edges(depot).empty()) {
//        cout << "Vertex is not connected" << endl;
        return {};
    }

    max_parameter_values.assign(num_parameters, -1);
    for (int e = 0; e < shrinked_graph.get_num_edges(); ++e) {
        for (int i = 0; i < num_parameters; ++i) {
            if (max_parameter_values[i] < shrinked_edge_parameters[e][i]) {
                max_parameter_values[i] = shrinked_edge_parameters[e][i];
            }
        }
    }

    // Indices of used edges and total parameter values
    deque<pair<deque<int>, vector<int>>> pareto_solutions;

    // Find initial solution through NN

    // Set pheromones
    double initial_pheromone = 2.0 / 2;    //1 / (nn[0]/nn[0] + nn[1]/nn[1])
    vector<vector<double>> pheromones(shrinked_graph.get_num_edges(), vector<double>(num_parameters, initial_pheromone));

    auto t1 = chrono::high_resolution_clock::now();

    // main loop
    int itr = 0;
    int full_num_iterations = 0;
    while (itr < iteration_limit) {
        itr += 1;
        full_num_iterations += 1;
        bool set_updated = false;
        deque<pair<deque<int>, vector<int>>> iteration_solutions;
        vector<pair<deque<int>, vector<int>>> candidate_solutions(num_ants);
        auto tt1 = chrono::high_resolution_clock::now();

        int ant_lambda_denominator = max(1, num_ants - 1);
#pragma omp parallel for num_threads(thread_count)
        for (int h = 0; h < num_ants; ++h) {
            double lambda1 = (double)h / ant_lambda_denominator;
            double lambda2 = 1 - lambda1;
            mt19937 ant_rng(rng_seed + (unsigned int)(full_num_iterations * num_ants + h));

            pair<deque<int>, vector<int>> candidate_solution = bicriterion_vrp_ant_crawl(pheromones, alpha, beta, lambda1, lambda2, exploration_probability, evaporation_rate, initial_pheromone, ant_rng);
            candidate_solutions[h] = std::move(candidate_solution);
        }

        for (auto &candidate_solution: candidate_solutions) {
            if (candidate_solution.first.empty()) {
                continue;
            }

            // Check the solution
            bool dominated = false;
            stack<int> to_remove;
            for (int closed_solution = 0; closed_solution < iteration_solutions.size(); closed_solution++) {
                if (check_domination(iteration_solutions[closed_solution].second, candidate_solution.second)) {
                    dominated = true;
                    break;
                } else if (check_domination(candidate_solution.second,
                                            iteration_solutions[closed_solution].second)) {
                    to_remove.push(closed_solution);
                }
            }

            if (!dominated) {
                // Update set for the new solution
                // Remove dominated ones
                while (!to_remove.empty()) {
                    int t = to_remove.top();
                    to_remove.pop();
                    iteration_solutions.erase(iteration_solutions.begin() + t);
                }
                // Add new one
                iteration_solutions.push_back(std::move(candidate_solution));
            }
        }
        auto tt2 = chrono::high_resolution_clock::now();
        double dur = chrono::duration_cast<chrono::nanoseconds>( tt2 - tt1 ).count();
        dur *= 1e-9;
//        cout << "Iteration duration: " << dur << endl;
        // Decrease pheromones
        for (int e = 0; e < shrinked_graph.get_num_edges(); ++e) {
            for (int i = 0; i < num_parameters; ++i) {
                pheromones[e][i] = (1 - evaporation_rate) * pheromones[e][i];
            }
        }
        // Local update
        for (auto &solution: iteration_solutions) {
            for (int e: solution.first) {
                for (int i = 0; i < num_parameters; ++i) {
                    pheromones[e][i] += (double) 1 / iteration_solutions.size();
                }
            }
        }
        // Update global solutions
        for (auto &solution: iteration_solutions) {
            bool dom = false;
            stack<int> to_remove;
            for (int closed_solution = 0; closed_solution < pareto_solutions.size(); closed_solution++) {
                bool left = false;
                bool right = false;
                if (check_domination(pareto_solutions[closed_solution].second, solution.second)) {
                    left = true;
                }
                else if (check_domination(solution.second, pareto_solutions[closed_solution].second)) {
                    right = true;
                }

                if (left && right) {
                    continue;
                }
                if (left) {
                    dom = true;
                    break;
                } else if (right) {
                    to_remove.push(closed_solution);
                }
            }
            if (dom) {
                continue;
            }
            else {
                // Update set for the new solution
                // Remove dominated ones
                while (!to_remove.empty()) {
                    auto t = to_remove.top();
                    to_remove.pop();
                    pareto_solutions.erase(pareto_solutions.begin() + t);
                }
                // Add new one
                pareto_solutions.push_back(solution);
                set_updated = true;

            }
        }
        iteration_solutions.clear();

        if (!pareto_solutions.empty()) {
            double average = 0;
            for (auto &s: pareto_solutions) {
                average += s.second[0];
            }
            average /= pareto_solutions.size();
        }

//        ofstream outfile;
//        string path = "vrp_iterations" + to_string(num_criticals) + ".csv";
//        outfile.open(path, ios::app);
//        outfile << average << endl;
//        outfile.close();

        if (set_updated) {
            itr = 0;
//            cout << "Solution improved (" << pareto_solutions[0].second[0] << ", " << pareto_solutions[0].second[1] << ")" << endl;
        }
//        else {
//            cout << "No improvement " << itr << endl;
//        }
    }

//    if (!pareto_solutions.empty())
//        save_route(pareto_solutions[0].first);

    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;
//    cout << "Optimization duration: " << duration << endl;

    answer_string = "";
    answer_string += to_string(shrinked_graph.get_num_edges()) + ",";

    answer_string += to_string(full_num_iterations) + "," + to_string(duration) + " ";

    for (auto &sol: pareto_solutions) {
        answer_string += "(" + to_string(sol.second[0]) + ";" + to_string(sol.second[1]) + "),";
    }

    return answer_string;
}

void VRPAntColony::save_route(deque<int> &route_edges) {
    vector<pair<double, double>> node_coords;

    ifstream coords_file;
    coords_file.open("D:/Work/data/finished_berlin_nodes.csv");
    string line;
    getline(coords_file, line);

    while (getline(coords_file, line, ',')) {
        getline(coords_file, line, ',');
        double latitude = stof(line);

        getline(coords_file, line, '\n');
        double longitude = stof(line);

        node_coords.emplace_back(latitude, longitude);
    }

    ofstream file;
    string file_path = "../traversed_distance_vrp_edges.csv";
    file.open(file_path);
    file << "latitude_1, longitude_1, latitude_2, longitude_2\n";

    for (int e : route_edges) {
        int start = shrinked_edge_nodes[e].first;
        int end = shrinked_edge_nodes[e].second;
        file << node_coords[graph.get_id_to_goal(start)].first << "," << node_coords[graph.get_id_to_goal(start)].second << "," << node_coords[graph.get_id_to_goal(end)].first << "," << node_coords[graph.get_id_to_goal(end)].second << "\n";
    }
    file.close();

    file.open("../goal_node_coords.csv");
    file << "latitude,longitude,id\n";
    for (int g = 0; g < num_criticals; ++g) {
        file << node_coords[graph.get_id_to_goal(g)].first << "," << node_coords[graph.get_id_to_goal(g)].second << "," << graph.get_id_to_goal(g) << "\n";
    }
    file.close();
}

pair<deque<int>, vector<int>> VRPAntColony::bicriterion_vrp_ant_crawl(vector<vector<double>>& pheromones, double& alpha, double& beta, double& lambda1, double& lambda2, double& exploration_probability, double& evaporation_rate, double& initial_pheromone, mt19937& rng) {
    // For all vehicles
    deque<int> route_edges;
    deque<int> route_nodes;
    vector<deque<int>> individual_routes;
    vector<int> total_parameters(num_parameters, 0);
    int goals_to_visit = num_criticals - 1;
    vector<bool> goal_visited(num_criticals, false);
    goal_visited[depot] = true;

    // For current vehicle
    int current_vehicle = 0;
    int current_vertex = depot;
    deque<int> current_nodes;
    deque<int> current_edges;
    current_nodes.push_back(depot);
    vector<int> current_parameters(num_parameters, 0);

    // Main loop
    bool search_finished = false;
    while (!search_finished) {
        // Build possible steps
        vector<int> next_edges_to_traverse;
        vector<int> out_edges = shrinked_graph.get_outgoing_edges(current_vertex);
        for (int e : out_edges) {
            // Check for visitation
            int next_vertex = shrinked_edge_nodes[e].second;
            if (goal_visited[next_vertex])
                continue;
            // Check energy feasibility
            int next_energy = add_energy(current_parameters[1], shrinked_edge_parameters[e][1]);
            bool allowed = false;
            for (int be: shrinked_graph.get_outgoing_edges(next_vertex)) {
                if (shrinked_edge_nodes[be].second == depot) {
                    if (battery_capacity - add_energy(next_energy, shrinked_edge_parameters[be][1]) >= 0) {
                        allowed = true;
                        break;
                    }
                }
            }
            if (allowed) {
                next_edges_to_traverse.push_back(e);
            }
        }

        // An unvisited vertex is available
        // Choose equally by edges ----------------------------
        if (!next_edges_to_traverse.empty()) {
            int chosen_edge = bicriterion_ant_pick_edge(next_edges_to_traverse, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
            int next_vertex = shrinked_edge_nodes[chosen_edge].second;
            // Make a transition
            current_vertex = next_vertex;

            current_nodes.push_back(current_vertex);
            current_edges.push_back(chosen_edge);

            current_parameters[0] += shrinked_edge_parameters[chosen_edge][0];
            current_parameters[1] = add_energy(current_parameters[1], shrinked_edge_parameters[chosen_edge][1]);
            goals_to_visit -= 1;
            goal_visited[current_vertex] = true;
        }
        // If no goals available, go to depot
        else {
            for (int e: shrinked_graph.get_outgoing_edges(current_vertex)) {
                if (shrinked_edge_nodes[e].second == depot) {
                    if (battery_capacity -
                        add_energy(current_parameters[1], shrinked_edge_parameters[e][1]) >= 0) {
                        next_edges_to_traverse.push_back(e);
                    }
                }
            }

//            cout << next_edges_to_traverse.size() << endl;
            if (next_edges_to_traverse.empty()) {
                return make_pair(deque<int>(), vector<int>());
            }

            int chosen_edge = bicriterion_ant_pick_edge(next_edges_to_traverse, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
            int next_vertex = shrinked_edge_nodes[chosen_edge].second;
            // Make a transition
            current_vertex = next_vertex;

            current_nodes.push_back(current_vertex);
            current_edges.push_back(chosen_edge);

            current_parameters[0] += shrinked_edge_parameters[chosen_edge][0];
            current_parameters[1] = add_energy(current_parameters[1], shrinked_edge_parameters[chosen_edge][1]);
            goal_visited[current_vertex] = true;

            // 2-opt heuristic
//            bicriterion_two_opt(current_nodes, current_edges, current_parameters, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rd);

            // Reset info
            for (int v: current_nodes) {
                route_nodes.push_back(v);
            }
            for (int e: current_edges) {
                route_edges.push_back(e);
            }

            current_vehicle += 1;
            for (int i = 0; i < num_parameters; ++i) {
                total_parameters[i] += current_parameters[i];
            }

            current_nodes.clear();
            current_nodes.push_back(depot);

            current_edges.clear();

            current_parameters.assign(num_parameters, 0);
        }

        if (goals_to_visit == 0 && current_vertex == depot) {
            search_finished = true;
        }

    }
    // Destination exchange heuristic
//    destination_bicriterion_exchange(route_nodes, route_edges, total_parameters, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rd);

    return make_pair(route_edges, total_parameters);
}


void VRPAntColony::bicriterion_two_opt(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                           vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                           double &lambda1, double &lambda2, mt19937 &rng) {
    deque<int> best_nodes;
    deque<int> best_edges;
    vector<int> best_parameters(num_parameters, INT_MAX);

    bool improved = true;

    while (improved) {
        improved = false;
        for (int v1 = 0; v1 < current_edges.size() - 2; ++v1) {
            for (int v2 = v1 + 2; v2 < current_edges.size(); ++v2) {
                deque<int> candidate_nodes(current_nodes);
                deque<int> candidate_edges(current_edges);
                vector<int> candidate_parameters(num_parameters, 0);

                // Reorder nodes
                int r1 = v1 + 1;
                int r2 = v2;
                while (r2 - r1 > 0) {
                    int r = candidate_nodes[r1];
                    candidate_nodes[r1] = candidate_nodes[r2];
                    candidate_nodes[r2] = r;
                    r1 += 1;
                    r2 -= 1;
                }

                // Repick edges
                for (int i = v1; i <= v2; ++i) {
                    int route_from = candidate_nodes[i];
                    int route_to = candidate_nodes[i + 1];
                    vector<int> possible_edges;
                    for (int e: shrinked_graph.get_outgoing_edges(route_from)) {
                        if (shrinked_edge_nodes[e].second == route_to) {
                            possible_edges.push_back(e);
                        }
                    }
                    candidate_edges[i] = bicriterion_ant_pick_edge(possible_edges, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
                }

                bool feasible = true;
                for (int e = 0; e < candidate_edges.size(); ++e) {
                    int new_energy = add_energy(candidate_parameters[1], shrinked_edge_parameters[candidate_edges[e]][1]);
                    if (new_energy > battery_capacity) {
                        feasible = false;
                        break;
                    }
                    candidate_parameters[0] += shrinked_edge_parameters[candidate_edges[e]][0];
                    candidate_parameters[1] = new_energy;
                }

                if (!feasible) {
                    continue;
                }

                if (check_strict_domination(candidate_parameters, best_parameters)) {
                    best_nodes = std::move(candidate_nodes);
                    best_edges = std::move(candidate_edges);
                    best_parameters = std::move(candidate_parameters);
                    improved = true;
                }
            }
        }

    }
    if (check_strict_domination(best_parameters, current_parameters)) {
//        cout << "Destination exchange: (" << current_parameters[0] << ", " << current_parameters[1] << ")  to  (" << best_parameters[0] << ", " << best_parameters[1] << ")" << endl;
        current_nodes = std::move(best_nodes);
        current_edges = std::move(best_edges);
        current_parameters = std::move(best_parameters);
    }
}


void VRPAntColony::bicriterion_destination_exchange(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                        vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                        double &lambda1, double &lambda2, mt19937 &rng) {
    deque<int> best_nodes;
    deque<int> best_edges;
    vector<int> best_parameters(num_parameters, INT_MAX);

    bool improved = true;

    while (improved) {
        improved = false;
        for (int v1 = 1; v1 < current_nodes.size() - 2; ++v1) {
            if (current_nodes[v1] == depot)
                continue;
            for (int v2 = v1 + 1; v2 < current_nodes.size() - 1; ++v2) {
                if (current_nodes[v2] == depot)
                    continue;
                deque<int> candidate_nodes(current_nodes);
                deque<int> candidate_edges(current_edges);
                vector<int> candidate_parameters(num_parameters, 0);

                int t = candidate_nodes[v1];
                candidate_nodes[v1] = candidate_nodes[v2];
                candidate_nodes[v2] = t;

                int before_v1 = candidate_nodes[v1 - 1];
                int new_v1 = candidate_nodes[v1];
                int after_v1 = candidate_nodes[v1 + 1];
                int before_v2 = candidate_nodes[v2 - 1];
                int new_v2 = candidate_nodes[v2];
                int after_v2 = candidate_nodes[v2 + 1];

                vector<int> edges_to_choose;
                for (int e: shrinked_graph.get_outgoing_edges(before_v1)) {
                    if (shrinked_edge_nodes[e].second == new_v1) {
                        edges_to_choose.push_back(e);
                    }
                }
                candidate_edges[v1 - 1] = bicriterion_ant_pick_edge(edges_to_choose, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);

                edges_to_choose.clear();
                for (int e: shrinked_graph.get_outgoing_edges(new_v1)) {
                    if (shrinked_edge_nodes[e].second == after_v1) {
                        edges_to_choose.push_back(e);
                    }
                }
                candidate_edges[v1] = bicriterion_ant_pick_edge(edges_to_choose, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);

                edges_to_choose.clear();
                for (int e: shrinked_graph.get_outgoing_edges(before_v2)) {
                    if (shrinked_edge_nodes[e].second == new_v2) {
                        edges_to_choose.push_back(e);
                    }
                }
                candidate_edges[v2 - 1] = bicriterion_ant_pick_edge(edges_to_choose, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);

                edges_to_choose.clear();
                for (int e: shrinked_graph.get_outgoing_edges(new_v2)) {
                    if (shrinked_edge_nodes[e].second == after_v2) {
                        edges_to_choose.push_back(e);
                    }
                }
                candidate_edges[v2] = bicriterion_ant_pick_edge(edges_to_choose, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);

                bool feasible = true;
                vector<int>candidate_route_parameters(num_parameters, 0);
                for (int e = 0; e < candidate_edges.size(); ++e) {
                    int new_energy = add_energy(candidate_route_parameters[1], shrinked_edge_parameters[candidate_edges[e]][1]);
                    if (new_energy > battery_capacity) {
                        feasible = false;
                        break;
                    }
                    candidate_route_parameters[0] += shrinked_edge_parameters[candidate_edges[e]][0];
                    candidate_route_parameters[1] = new_energy;
                    if (candidate_nodes[e + 1] == depot) {
                        for (int i = 0; i < num_parameters; ++i) {
                            candidate_parameters[i] += candidate_route_parameters[i];
                        }
                        candidate_route_parameters.assign(num_parameters, 0);
                    }
                }

                if (!feasible) {
                    continue;
                }

                if (check_strict_domination(candidate_parameters, best_parameters)) {
                    best_nodes = std::move(candidate_nodes);
                    best_edges = std::move(candidate_edges);
                    best_parameters = std::move(candidate_parameters);
                    improved = true;
                }
            }
        }

    }
    if (check_strict_domination(best_parameters, current_parameters)) {
//        cout << "Destination exchange: (" << current_parameters[0] << ", " << current_parameters[1] << ")  to  (" << best_parameters[0] << ", " << best_parameters[1] << ")" << endl;
        current_nodes = std::move(best_nodes);
        current_edges = std::move(best_edges);
        current_parameters = std::move(best_parameters);
    }
}


int VRPAntColony::bicriterion_ant_pick_edge(vector<int>& available_edges, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, mt19937& rng) {
    if (available_edges.empty()) {
        return -1;
    }
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    int num_possible_edges = available_edges.size();
    vector<double> probs(num_possible_edges);
    double sum = 0;
    for (int e = 0; e < num_possible_edges; ++e) {
//        double a = pow(pheromones[available_edges[e]], alpha);
//        double b = pow(1.0/((double)shrinked_graph.get_edge_parameters(available_edges[e])[0]), lambda1);
//        double c = pow(1.0/((double)shrinked_graph.get_edge_parameters(available_edges[e])[1]), lambda2);
        probs[e] = pow(pow(pheromones[available_edges[e]][0], lambda1) * pow(pheromones[available_edges[e]][1], lambda2), alpha) * pow((pow(1.0/((double)shrinked_edge_parameters[available_edges[e]][0]/max_parameter_values[0]), lambda1)
                                                                     * pow(1.0/((double)shrinked_edge_parameters[available_edges[e]][1]/max_parameter_values[1]), lambda2)), beta);
        probs[e] = min(probs[e], 1000000.0);
        sum += probs[e];

    }
    if (!isfinite(sum) || sum <= 0) {
        uniform_int_distribution<int> edge_distribution(0, num_possible_edges - 1);
        return available_edges[edge_distribution(rng)];
    }
    for (int e = 0; e < num_possible_edges; ++e) {
        probs[e] /= sum;
    }

    // Choose by probs
    double strat = unif(rng);
    int chosen_edge = -1;
    // Choose largest prob
    if (strat > exploration_probability) {
        double max_prob = -1;
        for (int e = 0; e < num_possible_edges; ++e) {
            if (probs[e] > max_prob) {
                max_prob = probs[e];
                chosen_edge = available_edges[e];
            }
        }


    }
        // Choose random prob
    else {
        double chosen_prob = unif(rng);
        double accumulated_prob = 0;
        for (int e = 0; e < num_possible_edges; ++e) {
            accumulated_prob += probs[e];
            if (accumulated_prob >= chosen_prob) {
                chosen_edge = available_edges[e];
                break;
            }
        }
    }

    if (chosen_edge == -1) {
        chosen_edge = available_edges.back();
    }

    return chosen_edge;
}


int VRPAntColony::add_energy(int& current_energy, int& added_energy) {
    int new_energy = current_energy + added_energy;
    if (new_energy > battery_capacity) {
        new_energy = battery_capacity + 1;
    } else if (new_energy < 0) {
        new_energy = 0;
    }
    return new_energy;
}
