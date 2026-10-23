#include "VRPAntColony.h"
#include "ShrinkGraph.h"
#include <chrono>
#include <stack>
#include <utility>
#include <random>
#include <climits>
#include <algorithm>
#include <cmath>
#include <omp.h>

VRPAntColony::VRPAntColony(Graph original_graph, int num_goals, int ants, int iterations,
                           int threads, int battery, int cover_k, unsigned int seed, int vehicles) {
    this->graph = std::move(original_graph);
    this->depot = 0;
    this->num_criticals = num_goals + 1;
    this->num_ants = ants;
    this->iteration_limit = iterations;
    this->thread_count = threads;
    this->max_vehicles = vehicles > 0 ? vehicles : num_goals;
    this->battery_capacity = battery;
    this->k_cover = cover_k;
    this->rng_seed = seed;
}

void VRPAntColony::k_path_cover(int k) {

    int num_nodes = graph.get_num_nodes();

    cover_id.assign(num_nodes, -1);
    vector<bool> in_cover(num_nodes, true);
    num_cover_nodes = 0;

    // Pick KPC nodes
    for (int n = 0; n < num_nodes; ++n) {

        // Terminals must remain endpoints after compression.
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

    outgoing_cover_edges.assign(num_cover_nodes, map<int, vector<int>>());
    incoming_cover_edges.assign(num_cover_nodes, map<int, vector<int>>());

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
                        continue;
                    }

                    while (!to_remove.empty()) {
                        int ind = to_remove.front();
                        to_remove.pop_front();
                        cover_params[next_node].erase(cover_params[next_node].begin() + ind);
                        cover_nodes[next_node].erase(cover_nodes[next_node].begin() + ind);
                    }
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

    }

}

map<int, vector<LabelPtr>> VRPAntColony::t_kpc_mls(int source){

    vector<set<LabelPtr, lex_compare>> closed_labels(num_cover_nodes);
    vector<set<LabelPtr, lex_compare>> open_labels(num_cover_nodes);
    vector<set<vector<int>>> t_sets(num_cover_nodes);
    set<LabelPtr, lex_compare> to_expand;
    auto source_label = make_shared<Label>(source, vector<int>(num_parameters, 0), nullptr, -1);

    to_expand.insert(source_label);
    open_labels[cover_id[source]].insert(source_label);

    while (!to_expand.empty()){
        LabelPtr current_label = *to_expand.begin();
        to_expand.erase(to_expand.begin());

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
                }

                auto new_label = make_shared<Label>(next_node, new_criteria, current_label, i);

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

    return bag;
}

bool VRPAntColony::check_domination(const vector<int>& v1, const vector<int>& v2) const{
    for (int i = 0; i < v1.size(); ++i) {
        if (v1[i] > v2[i])
            return false;
    }
    return true;
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
SolverRunResult VRPAntColony::bicriterion_ant_vrp_result() {
    SolverRunResult result;
    result.solver = "vrp";
    auto total_start = chrono::high_resolution_clock::now();
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
    result.preprocessing_seconds = s_dur;
    result.graph_nodes = shrinked_graph.get_num_nodes();
    result.graph_edges = shrinked_graph.get_num_edges();

    shrinked_edge_nodes.resize(shrinked_graph.get_num_edges());
    shrinked_edge_parameters.resize(shrinked_graph.get_num_edges());
    for (int i = 0; i < shrinked_graph.get_num_edges(); ++i) {
        shrinked_edge_nodes[i] = make_pair(shrinked_graph.get_edge_nodes(i).front(), shrinked_graph.get_edge_nodes(i).back());
        shrinked_edge_parameters[i] = shrinked_graph.get_edge_parameters(i);
    }

    for (int g = 0; g < num_criticals; ++g) {
        if (shrinked_graph.get_outgoing_edges(g).empty() || shrinked_graph.get_incoming_edges(g).empty()) {
            result.status = "disconnected";
            result.message = "a required vertex has no incoming or outgoing edge";
            result.total_seconds = chrono::duration<double>(chrono::high_resolution_clock::now() - total_start).count();
            return result;
        }
    }
    if (shrinked_graph.get_outgoing_edges(depot).empty() || shrinked_graph.get_incoming_edges(depot).empty()) {
        result.status = "disconnected";
        result.message = "the depot has no incoming or outgoing edge";
        result.total_seconds = chrono::duration<double>(chrono::high_resolution_clock::now() - total_start).count();
        return result;
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

    // Set pheromones
    double initial_pheromone = 2.0 / 2;
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
        // Workers write separate slots; pheromones stay read-only during construction.
        vector<pair<deque<int>, vector<int>>> candidate_solutions(num_ants);

        int ant_lambda_denominator = max(1, num_ants - 1);
#pragma omp parallel for num_threads(thread_count)
        for (int h = 0; h < num_ants; ++h) {
            double lambda1 = (double)h / ant_lambda_denominator;
            double lambda2 = 1 - lambda1;
            mt19937 ant_rng(rng_seed + (unsigned int)(full_num_iterations * num_ants + h));

            pair<deque<int>, vector<int>> candidate_solution = bicriterion_vrp_ant_crawl(pheromones, alpha, beta, lambda1, lambda2, exploration_probability, evaporation_rate, initial_pheromone, ant_rng);
            candidate_solutions[h] = std::move(candidate_solution);
        }

        // Merge in ant order so thread scheduling cannot reorder the Pareto updates.
        for (auto &candidate_solution: candidate_solutions) {
            if (candidate_solution.first.empty()) {
                continue;
            }

            int route_count = 0;
            string validation_error;
            if (!validate_solution(candidate_solution.first, candidate_solution.second,
                                   route_count, validation_error)) {
                result.invalid_candidates += 1;
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

        if (set_updated) {
            itr = 0;
        }
    }

    auto t2 = chrono::high_resolution_clock::now();

    double duration = chrono::duration_cast<chrono::nanoseconds>( t2 - t1 ).count();
    duration *= 1e-9;

    result.iterations = full_num_iterations;
    result.optimization_seconds = duration;

    sort(pareto_solutions.begin(), pareto_solutions.end(),
         [](const auto& lhs, const auto& rhs) {
             return lhs.second < rhs.second;
         });

    for (auto &sol: pareto_solutions) {
        int route_count = 0;
        string validation_error;
        if (validate_solution(sol.first, sol.second, route_count, validation_error)) {
            result.pareto.push_back(SolverParetoPoint{sol.second, route_count});
        } else {
            result.invalid_candidates += 1;
        }
    }

    result.status = result.pareto.empty() ? "no_solution" : "ok";
    if (result.pareto.empty()) {
        result.message = "no validated Pareto solution was produced";
    }
    result.total_seconds = chrono::duration<double>(chrono::high_resolution_clock::now() - total_start).count();
    return result;
}

string VRPAntColony::bicriterion_ant_vrp() {
    return bicriterion_ant_vrp_result().legacy_answer();
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

            // Reset info
            for (int v: current_nodes) {
                route_nodes.push_back(v);
            }
            for (int e: current_edges) {
                route_edges.push_back(e);
            }

            current_vehicle += 1;
            if (goals_to_visit > 0 && current_vehicle >= max_vehicles) {
                return make_pair(deque<int>(), vector<int>());
            }
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

    return make_pair(route_edges, total_parameters);
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

bool VRPAntColony::validate_solution(const deque<int>& route_edges,
                                     const vector<int>& objectives,
                                     int& route_count,
                                     string& reason) {
    route_count = 0;
    if (route_edges.empty() || objectives.size() < 2) {
        reason = "empty route or objective vector";
        return false;
    }

    vector<bool> visited(num_criticals, false);
    visited[depot] = true;
    int current_vertex = depot;
    int route_energy = 0;
    long total_distance = 0;
    long total_energy = 0;

    for (int edge: route_edges) {
        if (edge < 0 || edge >= shrinked_graph.get_num_edges()) {
            reason = "route contains an invalid edge id";
            return false;
        }
        if (shrinked_edge_nodes[edge].first != current_vertex) {
            reason = "route is not continuous";
            return false;
        }

        total_distance += shrinked_edge_parameters[edge][0];
        int added_energy = shrinked_edge_parameters[edge][1];
        route_energy = add_energy(route_energy, added_energy);
        if (route_energy > battery_capacity) {
            reason = "route exceeds battery capacity";
            return false;
        }

        current_vertex = shrinked_edge_nodes[edge].second;
        if (current_vertex > depot && current_vertex < num_criticals) {
            visited[current_vertex] = true;
        }
        if (current_vertex == depot) {
            total_energy += route_energy;
            route_energy = 0;
            route_count += 1;
        }
    }

    if (current_vertex != depot) {
        reason = "route does not return to the depot";
        return false;
    }
    if (route_count > max_vehicles) {
        reason = "route uses more vehicles than allowed";
        return false;
    }
    for (int goal = 1; goal < num_criticals; ++goal) {
        if (!visited[goal]) {
            reason = "route does not visit every goal";
            return false;
        }
    }
    if (total_distance != objectives[0] || total_energy != objectives[1]) {
        reason = "reported objectives do not match the route";
        return false;
    }
    return true;
}
