#include "SVRPAntColony.h"
#include <chrono>
#include <iostream>
#include <stack>
#include <utility>
#include <fstream>
#include <random>
#include <algorithm>
#include <climits>
#include <cmath>
#include <omp.h>
#include <unordered_map>

SVRPAntColony::SVRPAntColony(Graph original_graph, int num_goals, int ants, int iterations,
                             int threads, int battery, int cover_k, unsigned int seed) {
    this->graph = std::move(original_graph);
    this->num_criticals = num_goals + 1;
    this->num_ants = ants;
    this->iteration_limit = iterations;
    this->thread_count = threads;
    this->battery_capacity = battery;
    this->k_cover = cover_k;
    this->rng_seed = seed;
}

void SVRPAntColony::k_path_cover(int k) {
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

//    cout << "Number of cover edges after pruning -- " << num_cover_edges << endl;
//    cout << "% pruned edges -- " << ((float)(num_init_cover_edges - num_cover_edges) / (float)num_init_cover_edges) * 100 << endl;
//    cout << "Pruned by domination -- " << dom_pruned << endl;

//    cout << "Ended " << k << "-PC construction" << endl;
//    cout << endl;

}


map<int, vector<LabelPtr>> SVRPAntColony::t_kpc_mls(int source, bool expand_from_goals){
    auto t1 = chrono::high_resolution_clock::now();

    vector<vector<LabelPtr>> closed_labels(num_cover_nodes);
    vector<vector<LabelPtr>> open_labels(num_cover_nodes);
    vector<int> t_sets(num_cover_nodes, INT_MAX);
    set<LabelPtr, lex_compare> to_expand;
    long num_labels = 0;
    auto source_label = make_shared<Label>(source, vector<int>(num_parameters, 0), nullptr, -1);

    to_expand.insert(source_label);
    open_labels[cover_id[source]].push_back(source_label);

    int num_iters = 0;

    while (!to_expand.empty()){
        LabelPtr current_label = *to_expand.begin();
        to_expand.erase(to_expand.begin());

//        if (num_iters % 100000 == 0){
//            cout << to_expand.size() << endl;
//        }
        num_labels++;
        num_iters += 1;

        int current_node = current_label->node;
        vector<int> current_params = current_label->parameters;

        open_labels[cover_id[current_node]].erase(remove(open_labels[cover_id[current_node]].begin(), open_labels[cover_id[current_node]].end(), current_label));
        closed_labels[cover_id[current_node]].push_back(current_label);
        update_t_set(t_sets[cover_id[current_node]], current_params[1]);

        if (!expand_from_goals && (current_node != source && current_node <= num_criticals)) {
            continue;
        }

        for (auto& p : outgoing_cover_edges[cover_id[current_node]]){
            int next_node = p.first;

            if (current_label->parent && next_node == current_label->parent->node)
                continue;
            vector<int> param_vectors = p.second;       // Parameter vectors of possible routes to next_node
            for (int i = 0; i < param_vectors.size(); ++i){          // Every parameter vector produces a new label
                vector<int>& params = cover_edge_params[param_vectors[i]];
                vector<int> new_criteria = link_paths(current_params, params);

                // Check battery constraints
                if (battery_capacity - new_criteria[1] < 0) {
                    continue;
                }

                // Check for t-dominance by closed labels
                bool t_discarded = t_discard(t_sets[cover_id[next_node]], new_criteria);
                if (t_discarded)
                    continue;

                // Check if is dominated by or dominates open labels
                vector<LabelPtr>& other_labels = open_labels[cover_id[next_node]];
                bool is_dominated = false;
                bool dominates = false;
//                stack<LabelPtr> to_remove;
                for (LabelPtr& l : other_labels){
                    if (!dominates && check_domination(l->parameters, new_criteria)){
                        is_dominated = true;
                        break;
                    }
                    else if (check_domination(new_criteria, l->parameters)){
//                    to_remove.push(l);
                        to_expand.erase(l);
                        l = nullptr;
                        dominates = true;
                    }
                }

                if (is_dominated)
                    continue;

                other_labels.erase(remove_if(other_labels.begin(), other_labels.end(), [&](const LabelPtr lab)-> bool { return !lab; }), other_labels.end());

//                while (!to_remove.empty()){
//                    auto rit = to_remove.top();
//                    to_remove.pop();
//                    to_expand.erase(rit);
//                    other_labels.erase(rit);
//                    num_labels--;
//                }


                auto new_label = make_shared<Label>(next_node, new_criteria, current_label, i);
//                num_labels++;

                open_labels[cover_id[next_node]].push_back(new_label);
                to_expand.insert(new_label);
            }
        }
    }

    map<int, vector<LabelPtr>> bag;
    vector<bool> is_dock(graph.get_num_nodes(), false);

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

bool SVRPAntColony::check_domination(const vector<int>& v1, const vector<int>& v2) {
    return v1[0] <= v2[0] && v1[1] <= v2[1];
}

bool SVRPAntColony::epsilon_check_domination(const vector<int>& v1, const vector<int>& v2) const {
    return v1[0] * epsilon <= v2[0] && v1[1] * epsilon <= v2[1];
}

bool SVRPAntColony::check_soc_domination(const vector<int>& v1, const vector<int>& v2) {
    return v1[0] <= v2[0] && v1[1] <= v2[1] && v1[2] >= v2[2];
}

bool SVRPAntColony::check_strict_domination(const vector<int>& v1, const vector<int>& v2) {
    if ((v1[0] < v2[0]) && (v1[1] <= v2[1])) {
        return true;
    }
    if ((v1[0] <= v2[0]) && (v1[1] < v2[1])) {
        return true;
    }
    return false;
}

bool SVRPAntColony::check_soc_t_domination(const vector<int>& t, const vector<int>& v) {
    return t[0] <= v[1] && t[1] >= v[2];
}

bool SVRPAntColony::t_discard(int& t_set, vector<int>& new_vec){
    return t_set <= new_vec[1];
}

bool SVRPAntColony::epsilon_t_discard(int& t_set, vector<int>& new_vec) const{
    return t_set * epsilon <= new_vec[1];
}

bool SVRPAntColony::soc_t_discard(set<LabelPtr, soc_lex_compare>& closed_labels, set<vector<int>>& t_set, vector<int>& new_vec){
    // Check first parameter dominance
    if (closed_labels.empty())
        return false;
    auto& max_p = *(closed_labels.rbegin());

    if (new_vec[0] < max_p->parameters[0])
        return false;

    // Check dominance of t-set
    for (auto& lab : t_set){
        if (check_soc_t_domination(lab, new_vec)){
            return true;
        }
    }

    return false;
}


 void SVRPAntColony::update_t_set(int& t_set, int new_l) {
    if (new_l < t_set) {
        t_set = new_l;
    }
}

void SVRPAntColony::update_soc_t_set(set<vector<int>>& t_set, LabelPtr lab) {
    vector<int> new_vec(lab->parameters.begin() + 1, lab->parameters.end());
    for (const vector<int>& v : t_set){
        if (check_soc_domination(v, new_vec))
            return;
    }

    auto inserted = t_set.insert(new_vec);
    auto it = inserted.first;
    if (inserted.second){
        it++;
        vector<set<vector<int>>::iterator> to_remove;
        while(it != t_set.end()){
//            if (new_vec[0] <= (*it)[0] && new_vec[1] <= (*it)[1] && new_vec[2] >= (*it)[2]){
            if (check_soc_domination(new_vec, (*it))){
                to_remove.push_back(it);
            }
            it++;
        }

        for (auto i = to_remove.rbegin(); i != to_remove.rend(); ++i){
            t_set.erase(*i);
        }
    }
}

vector<int> SVRPAntColony::build_simple_timecons(int edge){
    return vector<int>({graph.get_edge_parameters(edge).first, graph.get_edge_parameters(edge).second});
}

vector<int> SVRPAntColony::build_simple_soc(vector<int> edge_params) const{
    int cons = edge_params[1];
    vector<int> new_soc({
                                max(0, cons),                                       // minSoC_before
                                cons,                                               // consumption
                                min(battery_capacity, battery_capacity - cons)});    // maxSoC_after

    return new_soc;
}

vector<int> SVRPAntColony::link_paths(vector<int>& current_timecons, vector<int>& suffix_timecons){
    return vector<int>({current_timecons[0] + suffix_timecons[0], add_energy(current_timecons[1], suffix_timecons[1]) });
}

vector<int> SVRPAntColony::link_soc_paths(vector<int>& current_soc, vector<int>& suffix_soc){
    vector<int> full_soc(3, 0);
    full_soc[0] = max(current_soc[0], current_soc[1] + suffix_soc[0]);
    full_soc[1] = max(current_soc[1] + suffix_soc[1], current_soc[0] - suffix_soc[2]);
    full_soc[2] = min(current_soc[2] - suffix_soc[1], suffix_soc[2]);
    return full_soc;
}


ShrinkGraph SVRPAntColony::shrink_graph() {

    if (cover_edge_params.empty())
        k_path_cover(k_cover);


    vector<deque<int>> shrink_edge_nodes;
    vector<vector<int>> shrink_edge_params;
    vector<vector<deque<int>>> shrink_edge_nodes_by_source(num_criticals);
    vector<vector<vector<int>>> shrink_edge_params_by_source(num_criticals);

#pragma omp parallel for num_threads(thread_count)
    for (int s = 0; s < num_criticals; ++s) {
        map<int, vector<LabelPtr>> paths = t_kpc_mls(s, false);

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

    in_connections.resize(num_criticals);
    out_connections.resize(num_criticals);
    for (auto &e: shrink_edge_nodes) {
        in_connections[e.back()].insert(e.front());
        out_connections[e.front()].insert(e.back());
    }

    return shrink_graph;
}

string SVRPAntColony::multicolony_ant_svrp() {
    // hyperparameters
    int num_colonies = 5;
    int num_ants_per_colony = num_ants;

    double evaporation_rate = 0.05;
    double exploration_probability = 0.05;
    double alpha = 1;
    double beta = 3;

    // Shrink graph
    auto s1 = chrono::high_resolution_clock::now();
    shrinked_graph = shrink_graph();
    auto s2 = chrono::high_resolution_clock::now();
    double s_dur = chrono::duration_cast<chrono::nanoseconds>( s2 - s1 ).count();
    s_dur *= 1e-9;
    answer_string += std::to_string(shrinked_graph.get_num_edges()) + "," + to_string(s_dur) + " ";

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

    energy_to_depot = backward_t_kpc_mls(depot);

    max_parameter_values.assign(num_parameters, -1);
    for (int e = 0; e < shrinked_graph.get_num_edges(); ++e) {
        for (int i = 0; i < num_parameters; ++i) {
            if (max_parameter_values[i] < shrinked_edge_parameters[e][i]) {
                max_parameter_values[i] = shrinked_edge_parameters[e][i];
            }
        }
    }

    // Indices of used edges and total parameter values
    deque<pair<deque<int>, vector<int>>> global_solutions;

    // Find initial solution through NN
//    vector<int> nearest_neighbor_params(num_parameters, 0);
//    int route_length = vrp_nearest_neighbor_heuristic(nearest_neighbor_params);

    // Set pheromones
    double initial_pheromone = 2.0 / 2;    //1 / (nn[0]/nn[0] + nn[1]/nn[1])
    vector<vector<vector<double>>> pheromones(num_colonies, vector<vector<double>>(shrinked_graph.get_num_edges(), vector<double>(num_parameters, initial_pheromone)));

    int full_num_iterations = 0;

    auto t1 = chrono::high_resolution_clock::now();

    // main loop
    int itr = 0;
    while (itr < iteration_limit) {
        itr += 1;
        full_num_iterations += 1;
        bool set_updated = false;

        auto tt1 = chrono::high_resolution_clock::now();

        deque<pair<int, pair<deque<int>, vector<int>>>> iteration_solutions;
        vector<vector<pair<deque<int>, vector<int>>>> candidate_solutions(num_colonies, vector<pair<deque<int>, vector<int>>>(num_ants_per_colony));

        int colony_lambda_denominator = max(1, num_ants_per_colony - 1);
#pragma omp parallel for num_threads(thread_count)
        for (int c = 0; c < num_colonies; ++c) {
            double interval_base = (double)c / (num_colonies + 1);
            double increment = (double)2 / (num_colonies + 1) / colony_lambda_denominator;
            for (int h = 0; h < num_ants_per_colony; ++h) {
//              cout << omp_get_thread_num() << endl;

                double lambda1 = interval_base + (h * increment);
                double lambda2 = 1 - lambda1;
                mt19937 ant_rng(rng_seed + (unsigned int)(full_num_iterations * num_colonies * num_ants_per_colony + c * num_ants_per_colony + h));

                pair<deque<int>, vector<int>> candidate_solution;
                candidate_solution = bicriterion_svrp_ant_crawl(pheromones[c], alpha, beta, lambda1, lambda2, exploration_probability, evaporation_rate, initial_pheromone, ant_rng);
                candidate_solutions[c][h] = std::move(candidate_solution);
            }
        }

        for (int c = 0; c < num_colonies; ++c) {
            for (auto &candidate_solution: candidate_solutions[c]) {
                if (candidate_solution.first.empty()) {
                    continue;
                }

                // Check the solution
                bool dominated = false;
                stack<int> to_remove;
                for (int closed_solution = 0; closed_solution < iteration_solutions.size(); closed_solution++) {
                    if (check_domination(iteration_solutions[closed_solution].second.second, candidate_solution.second)) {
                        dominated = true;
                        break;
                    } else if (check_domination(candidate_solution.second, iteration_solutions[closed_solution].second.second)) {
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
                    iteration_solutions.emplace_back(c, std::move(candidate_solution));
                }
            }
        }

        auto tt2 = chrono::high_resolution_clock::now();
        double dur = chrono::duration_cast<chrono::nanoseconds>( tt2 - tt1 ).count();
        dur *= 1e-9;
        cout << "Iteration duration: " << dur << endl;

        // Decrease pheromones
        for (int c = 0; c < num_colonies; ++c) {
            for (int e = 0; e < shrinked_graph.get_num_edges(); ++e) {
                for (int i = 0; i < num_parameters; ++i) {
                    pheromones[c][e][i] = (1 - evaporation_rate) * pheromones[c][e][i];
                }
            }
        }

        // Local update
        for (auto &solution: iteration_solutions) {
//            double penalty = (double)(solution.first.size() - (num_jumps))/solution.first.size();
            for (auto& p: solution.second.first) {
                for (int i = 0; i < num_parameters; ++i) {
                    pheromones[solution.first][p][i] += (double) 1 / iteration_solutions.size();
//                        pheromones[p.first][i] += (double) penalty / iteration_solutions.size();
                }

            }
        }
        // Update global solutions
        for (auto &solution: iteration_solutions) {
            bool dom = false;
            stack<int> to_remove;
            for (int closed_solution = 0; closed_solution < global_solutions.size(); closed_solution++) {
                bool left = false;
                bool right = false;
                if (check_domination(global_solutions[closed_solution].second, solution.second.second)) {
                    left = true;
                }
                else if (check_domination(solution.second.second, global_solutions[closed_solution].second)) {
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
                    global_solutions.erase(global_solutions.begin() + t);
                }
                // Add new one
                global_solutions.push_back(solution.second);
                set_updated = true;

            }
        }
        iteration_solutions.clear();

        if (!global_solutions.empty()) {
            double average = 0;
            for (auto &s: global_solutions) {
                average += s.second[0];
            }
            average /= global_solutions.size();

            ofstream outfile;
            string path = "svrp_iterations" + to_string(num_criticals) + ".csv";
            outfile.open(path, ios::app);
            outfile << average << endl;
            outfile.close();
        }

        if (set_updated) {
            itr = 0;
            cout << "Solution improved (" << global_solutions[0].second[0] << ", " << global_solutions[0].second[1] << ")" << endl;
        }
        else {
            cout << "No improvement " << itr << endl;
        }
    }

//    if (!global_solutions.empty())
//        save_route(global_solutions[0].first);

//    cout << exploitations << " exploitations and " << explorations << " explorations" << endl;
//    cout << num_steps << " steps and " << num_plans << " plans" << endl;

    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;
    cout << "Optimization duration: " << duration << endl;

    answer_string = "";

    answer_string += to_string(full_num_iterations) + "," + to_string(duration) + " ";

    for (auto &sol: global_solutions) {
        answer_string += "(" + to_string(sol.second[0]) + ";" + to_string(sol.second[1]) + "),";
    }

    return answer_string;
}

// Main solution function for kPC-STSP
string SVRPAntColony::bicriterion_ant_svrp() {
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

    energy_to_depot = backward_t_kpc_mls(depot);

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
//    vector<int> nearest_neighbor_params(num_parameters, 0);
//    int route_length = vrp_nearest_neighbor_heuristic(nearest_neighbor_params);

    // Set pheromones
    double initial_pheromone = 2.0 / 2;    //1 / (nn[0]/nn[0] + nn[1]/nn[1])
    vector<vector<double>> pheromones(shrinked_graph.get_num_edges(), vector<double>(num_parameters, initial_pheromone));

    int full_num_iterations = 0;

    auto t1 = chrono::high_resolution_clock::now();

    // main loop
    int itr = 0;
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
//            cout << omp_get_thread_num() << endl;

            double lambda1 = (double)h / ant_lambda_denominator;
            double lambda2 = 1 - lambda1;
            mt19937 ant_rng(rng_seed + (unsigned int)(full_num_iterations * num_ants + h));

            pair<deque<int>, vector<int>> candidate_solution;
            candidate_solution = bicriterion_svrp_ant_crawl(pheromones, alpha, beta, lambda1, lambda2, exploration_probability, evaporation_rate, initial_pheromone, ant_rng);
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
                } else if (check_domination(candidate_solution.second, iteration_solutions[closed_solution].second)) {
                    to_remove.push(closed_solution);
                }
            }

//                if (dominated) {
//                    continue;
//                } else {
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
//            double penalty = (double)(solution.first.size() - (num_jumps))/solution.first.size();
            for (auto& p: solution.first) {
                for (int i = 0; i < num_parameters; ++i) {
                    pheromones[p][i] += (double) 1 / iteration_solutions.size();
//                        pheromones[p.first][i] += (double) penalty / iteration_solutions.size();
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


        if (set_updated) {
            itr = 0;
        }
    }

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

void SVRPAntColony::save_route(deque<int> &route_edges) {
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
    string file_path = "../traversed_distance_svrp_edges.csv";
    file.open(file_path);
    file << "latitude_1, longitude_1, latitude_2, longitude_2\n";

    for (auto& e : route_edges) {
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

pair<deque<int>, vector<int>> SVRPAntColony::bicriterion_svrp_ant_crawl(vector<vector<double>>& pheromones, double& alpha, double& beta, double& lambda1, double& lambda2, double& exploration_probability, double& evaporation_rate, double& initial_pheromone, mt19937& rng) {
    // For all vehicles
    deque<int> route_edges;
    deque<int> route_nodes;

    double mutation_probability = 0.0;

    int num_nodes = num_criticals;
    int num_edges = shrinked_graph.get_num_edges();

    vector<int> total_parameters(num_parameters, 0);

    vector<int> goal_visited(num_nodes, 0);

    int goals_to_visit = num_criticals - 1;
    goal_visited[depot] = 1;

    // For current vehicle
    int current_vehicle = 0;
    int current_vertex = depot;
    deque<int> current_nodes;
    deque<int> current_edges;
    current_nodes.push_back(depot);
    vector<int> current_parameters(num_parameters, 0);

    uniform_real_distribution<double> mut_uni(0.0, 1.0);

    // Main loop
    bool search_finished = false;
    while (!search_finished) {
//        cout << goals_to_visit << endl;
        // Build possible steps
        vector<int> edges_to_unvisited;
        vector<int> edges_to_depot;
        // Reserve memory beforehand
        for (int out: shrinked_graph.get_outgoing_edges(current_vertex)) {
            //int next_vertex = shrinked_graph.get_edge_nodes(out).back();
            int next_vertex = shrinked_edge_nodes[out].second;
//            if (traversal_times >= 2)
//                continue;
            // Check for depot
            if (next_vertex == depot) {
                edges_to_depot.push_back(out);
                continue;
            }
            if (goal_visited[next_vertex] == 0) {
                // Check energy feasibility
                int next_energy = add_energy(current_parameters[1], shrinked_edge_parameters[out][1]);
                if (battery_capacity - next_energy >= energy_to_depot[cover_id[next_vertex]][0]) {
                    edges_to_unvisited.push_back(out);
                }
            }

        }

        // An unvisited vertex is available
        // Choose equally by edges ----------------------------
        if (!edges_to_unvisited.empty()) {
            int chosen_edge = bicriterion_ant_pick_edge(edges_to_unvisited, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
            int next_vertex = shrinked_edge_nodes[chosen_edge].second;
            // Make a transition
            current_vertex = next_vertex;
            current_nodes.push_back(current_vertex);
            current_edges.push_back(chosen_edge);
            current_parameters[0] += shrinked_edge_parameters[chosen_edge][0];
            current_parameters[1] = add_energy(current_parameters[1], shrinked_edge_parameters[chosen_edge][1]);
            goals_to_visit -= 1;
            goal_visited[current_vertex] += 1;

//            for (int v : in_connections[current_vertex]) {
//                out_times[v]--;
//            }

        }
            // Choose a previously visited vertex
            // Since no near vertices must be visited, use t-kPC-MLS and new heuristic to choose the most promising path
        else {
            vector<pair<deque<int>, vector<int>>> planned_routes = bicriterion_ant_plan(current_vertex, current_parameters,  goal_visited);
            vector<pair<deque<int>, vector<int>>> paths_to_goals;
            vector<pair<deque<int>, vector<int>>> paths_to_depot;

            for (auto &e: planned_routes) {
                if (shrinked_edge_nodes[e.first.back()].second == depot) {
                    paths_to_depot.push_back(e);
                }
                else {
                    paths_to_goals.push_back(e);
                }
            }
            if (current_vertex == depot) {
                // Jump to an unvisited vertex
//                vector<pair<deque<int>, vector<int>>> planned_routes = bicriterion_ant_plan(current_vertex, current_parameters,  goal_visited);
                auto chosen_path = bicriterion_ant_jump_overhead(paths_to_goals, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
                if (chosen_path.first.empty()) {
                    return make_pair(deque<int>(), vector<int>());
                }
                for (int e : chosen_path.first) {
                    current_nodes.push_back(shrinked_edge_nodes[e].second);
                    current_edges.push_back(e);
                    current_parameters[0] += shrinked_edge_parameters[e][0];
                    current_parameters[1] = add_energy(current_parameters[1], shrinked_edge_parameters[e][1]);

                    current_vertex = current_nodes.back();
                    goal_visited[current_vertex] += 1;
                }
                goals_to_visit -= 1;
            }
            // Jump
            else {
                if (!paths_to_goals.empty()) {
                    double prob = mut_uni(rng);
                    if (prob < mutation_probability) {
                        if (invert_subtour(current_nodes, current_edges, current_parameters, current_vertex, paths_to_goals, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng)) {
                            current_vertex = current_nodes.back();
                            continue;
                        }
                    }

                    auto chosen_path = bicriterion_ant_jump_overhead(paths_to_goals, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
                    if (chosen_path.first.empty()) {
                        return make_pair(deque<int>(), vector<int>());
                    }
                    for (int e : chosen_path.first) {
                        current_nodes.push_back(shrinked_edge_nodes[e].second);
                        current_edges.push_back(e);
                        current_parameters[0] += shrinked_edge_parameters[e][0];
                        current_parameters[1] = add_energy(current_parameters[1], shrinked_edge_parameters[e][1]);

                        current_vertex = current_nodes.back();
                        goal_visited[current_vertex] += 1;
                    }
                    goals_to_visit -= 1;
                }
                else {
                    auto chosen_path = bicriterion_ant_jump_overhead(paths_to_depot, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
                    if (chosen_path.first.empty()) {
                        return make_pair(deque<int>(), vector<int>());
                    }
                    for (int e : chosen_path.first) {
                        current_nodes.push_back(shrinked_edge_nodes[e].second);
                        current_edges.push_back(e);
                        current_parameters[0] += shrinked_edge_parameters[e][0];
                        current_parameters[1] = add_energy(current_parameters[1], shrinked_edge_parameters[e][1]);

                        current_vertex = current_nodes.back();
                        if (goal_visited[current_vertex] == 0) {
                            goals_to_visit--;
                        }
                        goal_visited[current_vertex] += 1;
                    }

                    current_nodes.pop_back();

                    current_vehicle += 1;
                    for (int i = 0; i < num_parameters; ++i) {
                        total_parameters[i] += current_parameters[i];
                    }
                    for (int v: current_nodes) {
                        route_nodes.push_back(v);
                    }
                    for (auto e: current_edges) {
                        route_edges.push_back(e);
                    }
                    current_nodes.clear();
                    current_nodes.push_back(depot);
                    current_edges.clear();
                    current_parameters.assign(num_parameters, 0);
                }
            }

        }

        if (goals_to_visit == 0 && current_vertex == depot) {
            search_finished = true;
        }

    }

    return make_pair(route_edges, total_parameters);
}

bool SVRPAntColony::invert_subtour(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters, int dead_end, const vector<pair<deque<int>, vector<int>>>& paths_to_goals,
                                   vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                   double &lambda1, double &lambda2, mt19937 &rng) {
    // Pick randomly reroute endpoint
    uniform_int_distribution<int> p_uni(0, paths_to_goals.size() - 1);
    auto possible_path = paths_to_goals[p_uni(rng)];
    deque<int> chosen_path = possible_path.first;
    vector<int> chosen_parameters = possible_path.second;
    int new_endpoint = shrinked_graph.get_edge_nodes(chosen_path.back())[0];
    if (new_endpoint == depot) {
        return false;
    }

    int ind = current_nodes.size() - 1;
    while (ind > 0 && current_nodes[ind] != new_endpoint) {
        ind--;
    }
    ind--;
    if (ind <= 0) {
        return false;
    }
    int new_origin = current_nodes[ind];

    deque<int> new_nodes;
    new_nodes.push_back(current_nodes[0]);
    deque<int> new_edges;
    new_edges.push_back(current_edges[0]);
    int current_node = current_nodes[1];
    int k = 1;
    while (k < ind) {
        new_nodes.push_back(current_node);
        new_edges.push_back(current_edges[k]);
        k++;
        current_node = current_nodes[k];
    }

    new_nodes.push_back(new_origin);

    // Try to find a direct edge between new origin and dead end
    vector<int> direct_edges;
    for (int e: shrinked_graph.get_outgoing_edges(new_origin)) {
        if (shrinked_edge_nodes[e].second == dead_end) {
            direct_edges.push_back(e);
        }
    }
    // There is a direct edge, build a route
    if (!direct_edges.empty()) {
//            cout << "Direct edge found" << endl;
        current_node = new_origin;
        int next_node = current_nodes.back();
        k = current_nodes.size() - 1;
        bool infeasible = false;
        while (current_node != new_endpoint) {
            vector<int> possible_edges;
            for (int e: shrinked_graph.get_outgoing_edges(current_node)) {
                if (shrinked_edge_nodes[e].second == next_node) {
                    possible_edges.push_back(e);
                }
            }
            if (possible_edges.empty()) {
//                    cout << "No edge for reverse route" << endl;
                infeasible = true;
                break;
            }
            new_edges.push_back(bicriterion_ant_pick_edge(possible_edges, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng));
            new_nodes.push_back(next_node);
            current_node = next_node;
            k--;
            next_node = current_nodes[k];
        }
        if (infeasible) {
            return false;
        }

        vector<int> new_parameters(num_parameters, 0);
        bool feasible = true;
        for (int new_edge : new_edges) {
            int new_energy = add_energy(new_parameters[1], shrinked_edge_parameters[new_edge][1]);
            if (new_energy > battery_capacity) {
                feasible = false;
                break;
            }
            new_parameters[0] += shrinked_edge_parameters[new_edge][0];
            new_parameters[1] = new_energy;
        }
        if (!feasible) {
//                cout << "Inverse route is infeasible" << endl;
            return false;
        }

        vector<int> potential_parameters = current_parameters;
        potential_parameters[0] += chosen_parameters[0];
        potential_parameters[1] = add_energy(potential_parameters[1], chosen_parameters[1]);
        if (check_strict_domination(potential_parameters, new_parameters)) {
//                cout << "Inverse route is dominated" << endl;
            return false;
        }

//        cout << new_parameters.size() << endl;

        current_nodes = std::move(new_nodes);
        current_edges = std::move(new_edges);
        current_parameters = std::move(new_parameters);
        cout << "Route inverted direct" << endl;
        return true;
    }

    return false;
}

vector<pair<deque<int>, vector<int>>> SVRPAntColony::plan_to_goal(int source, int goal, const vector<int>& source_parameters, set<int>& available_vertices) {
    auto t1 = chrono::high_resolution_clock::now();

    vector<pair<deque<int>, vector<int>>> routes;

    int num_nodes = shrinked_graph.get_num_nodes();

    unordered_map<int, vector<LabelPtr>> closed_labels;
    unordered_map<int, vector<LabelPtr>> open_labels;
    unordered_map<int, int> t_sets;
    for (int l: available_vertices) {
        closed_labels[l] = vector<LabelPtr>();
        open_labels[l] = vector<LabelPtr>();
        t_sets[l] = INT_MAX;
    }
    set<LabelPtr, lex_compare> to_expand;
    long num_labels = 0;
    auto source_label = make_shared<Label>(source, source_parameters, nullptr, -1);
//    num_labels++;

    to_expand.insert(source_label);
    open_labels[source].push_back(source_label);

    int num_iters = 0;

    while (!to_expand.empty()){
        LabelPtr current_label = *to_expand.begin();
        to_expand.erase(to_expand.begin());


        num_iters += 1;

        num_labels++;

        int current_node = current_label->node;
        vector<int> current_params = current_label->parameters;

        open_labels[current_node].erase(remove(open_labels[current_node].begin(), open_labels[current_node].end(), current_label));
        closed_labels[current_node].push_back(current_label);
        update_t_set(t_sets[current_node], current_params[1]);

        vector<int> new_criteria(num_parameters);

        for (int e : shrinked_graph.get_outgoing_edges(current_node)){
            int next_node = shrinked_edge_nodes[e].second;
            if (available_vertices.find(next_node) == available_vertices.end()) {
                continue;
            }

            if (current_label->parent && next_node == current_label->parent->node){
                continue;
            }

            new_criteria[0] = current_params[0] + shrinked_edge_parameters[e][0];
            new_criteria[1] = max(0, current_params[1] + shrinked_edge_parameters[e][1]);

            if (t_discard(t_sets[goal], new_criteria)) {
                continue;
            }

            // Check energy feasibility
            if (battery_capacity - new_criteria[1] < energy_to_depot[cover_id[next_node]][0]) {
                continue;
            }

            // Check for t-dominance by closed labels
            bool t_discarded = t_discard(t_sets[next_node], new_criteria);
            if (t_discarded)
                continue;

            // Check if is dominated by or dominates open labels
            bool dominates = false;
            vector<LabelPtr>& other_labels = open_labels[next_node];
            bool is_dominated = false;
            for (LabelPtr& l : other_labels){
                if (!dominates && check_domination(l->parameters, new_criteria)){
                    is_dominated = true;
                    break;
                }
                else if (check_domination(new_criteria, l->parameters)){
//                    to_remove.push(l);
                    to_expand.erase(l);
                    l = nullptr;
                    dominates = true;
                }
            }

            if (is_dominated)
                continue;

            other_labels.erase(remove_if(other_labels.begin(), other_labels.end(), [&](const LabelPtr lab)-> bool { return !lab; }), other_labels.end());

            auto new_label = make_shared<Label>(next_node, new_criteria, current_label, e);

            open_labels[next_node].push_back(new_label);
            to_expand.insert(new_label);
        }
    }

    for (auto& current_label : closed_labels[goal]) {
        deque<int> used_edges;
        vector<int> used_params = source_parameters;
        LabelPtr cur = current_label;
        while (cur->node != source) {
            used_edges.push_front(cur->used_edge);
            cur = cur->parent;
        }
        bool invalid = false;
        for (int e: used_edges) {
            used_params[0] += shrinked_edge_parameters[e][0];
            used_params[1] = max(0, used_params[1] + shrinked_edge_parameters[e][1]);
            if (used_params[1] > battery_capacity) {
                invalid = true;
                break;
            }
        }
        if (invalid) {
            continue;
        }
        routes.emplace_back(used_edges, used_params);
    }


    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;

    return routes;
}

pair<deque<int>, vector<int>> SVRPAntColony::bicriterion_ant_jump_overhead(const vector<pair<deque<int>, vector<int>>>& paths_to_goals,
                                                  vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                                  double &lambda1, double &lambda2, mt19937 &rng) {
    int num_choices = paths_to_goals.size();
    if (num_choices == 0) {
        return make_pair(deque<int>(), vector<int>());
    }
    vector<double> probs(num_choices);

    double sum = 0;
    for (int e = 0; e < num_choices; ++e) {

        // Calculate average pheromones
        deque<int> candidate_edges = paths_to_goals[e].first;
        vector<int> candidate_parameters = paths_to_goals[e].second;
        vector<double> mean_pheromones(num_parameters, 0);
        for (int edge: candidate_edges) {
            mean_pheromones[0] += pheromones[edge][0];
            mean_pheromones[1] += pheromones[edge][1];

        }

        mean_pheromones[0] /= candidate_edges.size();
        mean_pheromones[1] /= candidate_edges.size();

        probs[e] = pow(pow(mean_pheromones[0], lambda1) * pow(mean_pheromones[1], lambda2), alpha) * pow((pow(1.0/((double)candidate_parameters[0]/max_parameter_values[0]), lambda1)
                                                                                                 * pow(1.0/((double)candidate_parameters[1]/max_parameter_values[1]), lambda2)), beta);

        sum += probs[e];
    }
    if (!isfinite(sum) || sum <= 0) {
        uniform_int_distribution<int> path_distribution(0, num_choices - 1);
        return paths_to_goals[path_distribution(rng)];
    }
    for (int e = 0; e < num_choices; ++e) {
        probs[e] /= sum;
    }

    std::uniform_real_distribution<double> unif(0.0, 1.0);

    // Choose by probs
    double strat = unif(rng);
    int chosen_path = -1;
    // Choose largest prob
    if (strat > exploration_probability) {
        double max_prob = -1;
        for (int e = 0; e < num_choices; ++e) {
            if (probs[e] > max_prob) {
                max_prob = probs[e];
                chosen_path = e;
            }
        }
    }
        // Choose random prob
    else {
        double chosen_prob = unif(rng);
        double accumulated_prob = 0;
        for (int e = 0; e < num_choices; ++e) {
            accumulated_prob += probs[e];
            if (accumulated_prob >= chosen_prob) {
                chosen_path = e;
                break;
            }
        }
    }

    if (chosen_path == -1) {
        chosen_path = num_choices - 1;
    }

    return paths_to_goals[chosen_path];
}


vector<pair<deque<int>, vector<int>>> SVRPAntColony::bicriterion_ant_plan(int source, vector<int> source_parameters, vector<int>& goal_visited) {
    auto t1 = chrono::high_resolution_clock::now();

    vector<pair<deque<int>, vector<int>>> routes;
    if (source != depot)
        goal_visited[depot] = 0;

    int num_nodes = shrinked_graph.get_num_nodes();

    vector<vector<LabelPtr>> closed_labels(num_nodes);
    vector<vector<LabelPtr>> open_labels(num_nodes);
    vector<int> t_sets(num_nodes, INT_MAX);
    set<LabelPtr, lex_compare> to_expand;
    long num_labels = 0;
    auto source_label = make_shared<Label>(source, std::move(source_parameters), nullptr, -1);
//    num_labels++;

    to_expand.insert(source_label);
    open_labels[source].push_back(source_label);

    int num_iters = 0;

    while (!to_expand.empty()){
        LabelPtr current_label = *to_expand.begin();
        to_expand.erase(to_expand.begin());

//        if (num_iters % 100000 == 0){
//            cout << to_expand.size() << endl;
//        }
        num_iters += 1;

        num_labels++;

        int current_node = current_label->node;
        vector<int> current_params = current_label->parameters;

        open_labels[current_node].erase(remove(open_labels[current_node].begin(), open_labels[current_node].end(), current_label));
        closed_labels[current_node].push_back(current_label);
        update_t_set(t_sets[current_node], current_params[1]);

        if (goal_visited[current_node] == 0) {
            deque<int> used_edges;
            vector<int> used_params = current_params;
            LabelPtr cur = current_label;
            while (cur->node != source) {
                used_edges.push_front(cur->used_edge);
                cur = cur->parent;
            }
            routes.emplace_back(used_edges, used_params);
            continue;
        }

        vector<int> new_criteria(num_parameters);
//        stack<LabelPtr> to_remove;

        for (int e : shrinked_graph.get_outgoing_edges(current_node)){
            int next_node = shrinked_edge_nodes[e].second;

            if (current_label->parent && next_node == current_label->parent->node)
                continue;

            //vector<int> new_criteria = link_paths(current_params, shrinked_edge_parameters[e]);

            new_criteria[0] = current_params[0] + shrinked_edge_parameters[e][0];
            new_criteria[1] = max(0, current_params[1] + shrinked_edge_parameters[e][1]);

            // Check energy feasibility
            if (battery_capacity - new_criteria[1] < energy_to_depot[cover_id[next_node]][0]) {
                continue;
            }

            // Check for t-dominance by closed labels
            bool t_discarded = epsilon_t_discard(t_sets[next_node], new_criteria);
            if (t_discarded)
                continue;

            // Check if is dominated by or dominates open labels


            bool dominates = false;
            vector<LabelPtr>& other_labels = open_labels[next_node];
            bool is_dominated = false;
            for (LabelPtr& l : other_labels){
                if (!dominates && epsilon_check_domination(l->parameters, new_criteria)){
                    is_dominated = true;
                    break;
                }
                else if (epsilon_check_domination(new_criteria, l->parameters)){
//                    to_remove.push(l);
                    to_expand.erase(l);
                    l = nullptr;
                    dominates = true;
                }
            }

            if (is_dominated)
                continue;

            other_labels.erase(remove_if(other_labels.begin(), other_labels.end(), [&](const LabelPtr lab)-> bool { return !lab; }), other_labels.end());

//            while (!to_remove.empty()){
//                auto rit = to_remove.top();
//                to_remove.pop();
//                to_expand.erase(rit);
//                other_labels.erase(rit);
//                num_labels--;
//            }

            auto new_label = make_shared<Label>(next_node, new_criteria, current_label, e);
//            num_labels++;

            open_labels[next_node].push_back(new_label);
            to_expand.insert(new_label);
        }
    }

    goal_visited[depot] = 1;

    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;

    return routes;
}

void SVRPAntColony::bicriterion_two_opt(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
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

                bool unable = false;

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
                    if (possible_edges.empty()) {
                        unable = true;
                        break;
                    }
                    candidate_edges[i] = bicriterion_ant_pick_edge(possible_edges, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);
                }

                if (unable) {
                    continue;
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
        current_nodes = std::move(best_nodes);
        current_edges = std::move(best_edges);
        current_parameters = std::move(best_parameters);
    }
}

void SVRPAntColony::vertex_reinsertion(deque<int> &current_nodes, deque<int> &current_edges, vector<int> &current_parameters,
                                       vector<vector<double>> &pheromones, double &exploration_probability, double &alpha, double &beta,
                                       double &lambda1, double &lambda2, mt19937 &rng) {
    deque<int> best_nodes;
    deque<int> best_edges;
    vector<int> best_parameters(num_parameters, INT_MAX);

    bool improved = true;

    while (improved) {
        improved = false;
        // Node to reinsert
        for (int v = 1; v < current_nodes.size() - 1; ++v) {

            int vertex_to_insert = current_nodes[v];
            int vertex_before = current_nodes[v - 1];
            int vertex_after = current_nodes[v + 1];

            vector<int> possible_edges;
            for (int e: shrinked_graph.get_outgoing_edges(vertex_before)) {
                if (shrinked_edge_nodes[e].second == vertex_after) {
                    possible_edges.push_back(e);
                }
            }
            if (possible_edges.empty()) {
                continue;
            }

            int replacement_edge = bicriterion_ant_pick_edge(possible_edges, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng);

            // Reinsert after this position
            for (int t = 1; t < current_nodes.size(); ++t) {
                if (v == t) {
                    continue;
                }
                int insertion_position = t;
                if (t > v) {
                    insertion_position--;
                }
                deque<int> candidate_nodes(current_nodes);
                deque<int> candidate_edges(current_edges);
                vector<int> candidate_parameters(num_parameters, 0);

                // Bind the gap
                candidate_edges[v - 1] = replacement_edge;
                candidate_edges.erase(candidate_edges.begin() + v);
                candidate_nodes.erase(candidate_nodes.begin() + v);

                // Insert vertex
                candidate_nodes.insert(candidate_nodes.begin() + insertion_position, vertex_to_insert);
                // Remove binding edge
                candidate_edges.erase(candidate_edges.begin() + insertion_position);

                int bind_from = candidate_nodes[insertion_position - 1];
                int bind_to = candidate_nodes[insertion_position + 1];

                possible_edges.clear();
                for (int e: shrinked_graph.get_outgoing_edges(vertex_to_insert)) {
                    if (shrinked_edge_nodes[e].second == bind_to) {
                        possible_edges.push_back(e);
                    }
                }
                if (possible_edges.empty()) {
                    continue;
                }

                candidate_edges.insert(candidate_edges.begin() + insertion_position, bicriterion_ant_pick_edge(possible_edges, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng));

                possible_edges.clear();
                for (int e: shrinked_graph.get_outgoing_edges(bind_from)) {
                    if (shrinked_edge_nodes[e].second == vertex_to_insert) {
                        possible_edges.push_back(e);
                    }
                }
                if (possible_edges.empty()) {
                    continue;
                }

                candidate_edges.insert(candidate_edges.begin() + insertion_position, bicriterion_ant_pick_edge(possible_edges, pheromones, exploration_probability, alpha, beta, lambda1, lambda2, rng));

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
                    //v = -1;
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

int SVRPAntColony::bicriterion_ant_pick_edge(vector<int>& available_edges, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, mt19937& rng) {
    if (available_edges.empty()) {
        return -1;
    }
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    int num_possible_edges = available_edges.size();
    vector<double> probs(num_possible_edges);
    double sum = 0;
    for (int e = 0; e < num_possible_edges; ++e) {
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

int SVRPAntColony::bicriterion_ant_pick_overhead(vector<pair<deque<int>, vector<int>>>& planned_routes, vector<vector<double>>& pheromones, double& exploration_probability, double& alpha, double& beta, double& lambda1, double& lambda2, mt19937& rng) {

    int num_possible_edges = planned_routes.size();
    if (num_possible_edges == 0) {
        return -1;
    }
    vector<double> probs(num_possible_edges);

    double sum = 0;
    for (int e = 0; e < num_possible_edges; ++e) {
//        probs[e] = pow(pow(pheromones[available_edges[e]][0], lambda1) * pow(pheromones[available_edges[e]][1], lambda2), alpha) * pow((pow(1.0/((double)shrinked_graph.get_edge_parameters(available_edges[e])[0]/max_parameter_values[0]), lambda1)
//                                                                                                                                        * pow(1.0/((double)shrinked_graph.get_edge_parameters(available_edges[e])[1]/max_parameter_values[1]), lambda2)), beta);
        // Calculate average pheromones
        deque<int> current_edges = planned_routes[e].first;
        vector<int> current_parameters = planned_routes[e].second;
        vector<double> mean_pheromones(num_parameters, 0);
        for (int edge: current_edges) {
            for (int i = 0; i < num_parameters; ++i) {
                mean_pheromones[i] += pheromones[edge][i];
            }
        }
        for (int i = 0; i < num_parameters; ++i) {
            mean_pheromones[i] /= current_edges.size();
        }

        probs[e] = pow(pow(mean_pheromones[0], lambda1) * pow(mean_pheromones[1], lambda2), alpha) * pow((pow(1.0/((double)current_parameters[0]/max_parameter_values[0]), lambda1) *
                                                                                                          pow(1.0/((double)current_parameters[1]/max_parameter_values[1]), lambda2)), beta);
        probs[e] = min(probs[e], 1000000.0);
        sum += probs[e];
    }
    if (!isfinite(sum) || sum <= 0) {
        uniform_int_distribution<int> route_distribution(0, num_possible_edges - 1);
        return route_distribution(rng);
    }
    for (int e = 0; e < num_possible_edges; ++e) {
        probs[e] /= sum;
    }

    std::uniform_real_distribution<double> unif(0.0, 1.0);

    // Choose by probs
    double strat = unif(rng);
    int next_vertex = -1;
    int chosen_edge = -1;
    // Choose largest prob
    if (strat > exploration_probability) {
        double max_prob = -1;
        for (int e = 0; e < num_possible_edges; ++e) {
            if (probs[e] > max_prob) {
                max_prob = probs[e];
                chosen_edge = e;
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
                chosen_edge = e;
                break;
            }
        }
    }
    if (chosen_edge == -1) {
        chosen_edge = num_possible_edges - 1;
    }
    return chosen_edge;
}


int SVRPAntColony::add_energy(int& current_energy, int& added_energy) {
    return max(0, current_energy + added_energy);
}

void SVRPAntColony::print_out_route_coords(deque<int> &route) {
    vector<pair<double, double>> node_coords;

    ifstream coords_file;
    coords_file.open("../../data/merged_berlin_nodes.csv");
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
    string file_path = "../nn_route.csv";
    file.open(file_path);
    file << "latitude_1, longitude_1, latitude_2, longitude_2\n";
    for (int i = 0; i < route.size() - 1; ++i) {
        int start = route[i];
        int end = route[i + 1];
        file << node_coords[start].first << "," << node_coords[start].second << "," << node_coords[end].first << "," << node_coords[end].second << "\n";
    }

    file.close();

    file.open("../goal_node_coords.csv");
    file << "latitude,longitude,id\n";
    for (int g = 0; g < num_criticals; ++g) {
        file << node_coords[g].first << "," << node_coords[g].second << "," << g << "\n";
    }
    file.close();
}


// Use backward t-kPC-MLS from depot to compute energies (only minimum energy)
vector<vector<int>> SVRPAntColony::backward_t_kpc_mls(int source) {
    //kPC already computed
    auto t1 = chrono::high_resolution_clock::now();

    vector<set<LabelPtr, soc_lex_compare>> closed_labels(num_cover_nodes);
    vector<set<LabelPtr, soc_lex_compare>> open_labels(num_cover_nodes);
    vector<set<vector<int>>> t_sets(num_cover_nodes);
    set<LabelPtr, soc_lex_compare> to_expand;
    long num_labels = 0;
    auto source_label = make_shared<Label>(source, vector<int>({ 0, 0, battery_capacity}), nullptr, -1);
    num_labels++;

    to_expand.insert(source_label);
    open_labels[cover_id[source]].insert(source_label);

    int num_iters = 0;

    while (!to_expand.empty()){
        LabelPtr current_label = *to_expand.begin();
        to_expand.erase(to_expand.begin());

//        if (num_iters % 100000 == 0){
//            cout << to_expand.size() << endl;
//        }
        num_iters += 1;

        int current_node = current_label->node;
        vector<int> current_params = current_label->parameters;

        open_labels[cover_id[current_node]].erase(current_label);
        closed_labels[cover_id[current_node]].insert(current_label);
        update_soc_t_set(t_sets[cover_id[current_node]], current_label);

        for (auto& p : incoming_cover_edges[cover_id[current_node]]){
            int next_node = p.first;

            if (current_label->parent && next_node == current_label->parent->node)
                continue;
            vector<int> param_vectors = p.second;       // Parameter vectors of possible routes to next_node
            for (int i = 0; i < param_vectors.size(); ++i){          // Every parameter vector produces a new label
                vector<int> params = build_simple_soc(cover_edge_params[param_vectors[i]]);
                vector<int> new_criteria = link_soc_paths(params, current_params);

                // Check for t-dominance by closed labels
                bool t_discarded = soc_t_discard(closed_labels[cover_id[next_node]], t_sets[cover_id[next_node]], new_criteria);
                if (t_discarded)
                    continue;

                // Check if is dominated by or dominates open labels
                set<LabelPtr, soc_lex_compare>& other_labels = open_labels[cover_id[next_node]];
                bool is_dominated = false;
                stack<LabelPtr> to_remove;
                for (LabelPtr l : other_labels){
                    if (check_soc_domination(l->parameters, new_criteria)){
                        is_dominated = true;
                        break;
                    }
                    else if (check_soc_domination(new_criteria, l->parameters)){
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

    vector<vector<int>> solution_bag(num_cover_nodes);
    for (int i = 0; i < num_cover_nodes; ++i) {
        if (!closed_labels[i].empty()) {
            solution_bag[i] = (*closed_labels[i].begin())->parameters;
        }
    }

    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;
//    cout << "Backward energy query duration: " << duration << endl;

    return solution_bag;
}
