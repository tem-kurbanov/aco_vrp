#include "Graph.h"
#include "SVRPAntColony.h"
#include "VRPAntColony.h"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;

enum class SolverMode {
    Both,
    Svrp,
    Vrp
};

struct InputGraph {
    int num_nodes = 0;
    vector<pair<int, int>> edge_points;
    vector<pair<int, int>> edge_parameters;
};

struct ProgramOptions {
    string graph_path;
    int num_goals = -1;
    vector<int> fixed_goals;
    optional<int> depot;
    int num_tests = 10;
    optional<unsigned int> seed;
    SolverMode solver = SolverMode::Both;
    int ants = 100;
    int iterations = 20;
    int threads = 5;
    int battery = 50000;
    int k_cover = 24;
    string output_path;
};

string trim(const string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

int parse_int_field(const string& field, int line_number) {
    string cleaned = trim(field);
    size_t parsed = 0;
    int value = 0;
    try {
        value = stoi(cleaned, &parsed);
    } catch (const exception&) {
        throw runtime_error("Invalid integer on line " + to_string(line_number) + ": '" + field + "'");
    }
    if (parsed != cleaned.size()) {
        throw runtime_error("Invalid integer on line " + to_string(line_number) + ": '" + field + "'");
    }
    return value;
}

int parse_int_arg(const string& value, const string& name) {
    try {
        return parse_int_field(value, 0);
    } catch (const exception&) {
        throw runtime_error("Invalid value for " + name + ": '" + value + "'");
    }
}

unsigned int parse_seed_arg(const string& value) {
    int seed = parse_int_arg(value, "--seed");
    if (seed < 0) {
        throw runtime_error("--seed must be non-negative");
    }
    return static_cast<unsigned int>(seed);
}

vector<int> parse_int_csv_line(const string& line, int line_number) {
    vector<int> values;
    string field;
    stringstream stream(line);

    while (getline(stream, field, ',')) {
        values.push_back(parse_int_field(field, line_number));
    }

    if (!line.empty() && line.back() == ',') {
        throw runtime_error("Trailing comma on line " + to_string(line_number));
    }

    return values;
}

vector<int> parse_int_list(const string& text, const string& source_name) {
    string normalized = text;
    for (char& ch: normalized) {
        if (ch == ',' || ch == ';' || isspace(static_cast<unsigned char>(ch))) {
            ch = ' ';
        }
    }

    vector<int> values;
    string token;
    stringstream stream(normalized);
    while (stream >> token) {
        values.push_back(parse_int_arg(token, source_name));
    }

    if (values.empty()) {
        throw runtime_error(source_name + " did not contain any ids");
    }

    return values;
}

vector<int> read_goal_ids_file(const string& path) {
    ifstream file(path);
    if (!file) {
        throw runtime_error("Cannot open goals file: " + path);
    }

    stringstream buffer;
    buffer << file.rdbuf();
    return parse_int_list(buffer.str(), path);
}

InputGraph read_graph(const string& graph_path) {
    ifstream graph_file(graph_path);
    if (!graph_file) {
        throw runtime_error("Cannot open graph file: " + graph_path);
    }

    string line;
    int line_number = 1;
    if (!getline(graph_file, line)) {
        throw runtime_error("Graph file is empty: " + graph_path);
    }

    vector<int> metadata = parse_int_csv_line(line, line_number);
    if (metadata.size() != 2) {
        throw runtime_error("Expected first line to contain '<num_nodes>,<num_edges>'");
    }

    InputGraph graph;
    graph.num_nodes = metadata[0];
    const int expected_edges = metadata[1];
    if (graph.num_nodes <= 0) {
        throw runtime_error("Number of nodes must be positive");
    }
    if (expected_edges < 0) {
        throw runtime_error("Number of edges cannot be negative");
    }

    line_number += 1;
    if (!getline(graph_file, line)) {
        throw runtime_error("Missing CSV header line");
    }

    while (getline(graph_file, line)) {
        line_number += 1;
        if (trim(line).empty()) {
            continue;
        }

        vector<int> row = parse_int_csv_line(line, line_number);
        if (row.size() != 4) {
            throw runtime_error("Expected 4 columns on line " + to_string(line_number));
        }

        int start = row[0];
        int end = row[1];
        int p1 = row[2];
        int p2 = row[3];

        if (start < 0 || start >= graph.num_nodes || end < 0 || end >= graph.num_nodes) {
            throw runtime_error("Edge endpoint outside node range on line " + to_string(line_number));
        }

        graph.edge_points.emplace_back(start, end);
        graph.edge_parameters.emplace_back(p1, p2);
    }

    if ((int)graph.edge_points.size() != expected_edges) {
        throw runtime_error(
                "Expected " + to_string(expected_edges) + " edges, read " + to_string(graph.edge_points.size()));
    }

    return graph;
}

bool is_option(const string& value) {
    return value.rfind("--", 0) == 0;
}

string require_value(int& index, int argc, char* argv[], const string& option) {
    if (index + 1 >= argc || is_option(argv[index + 1])) {
        throw runtime_error("Missing value for " + option);
    }
    index += 1;
    return argv[index];
}

SolverMode parse_solver(const string& value) {
    if (value == "both") {
        return SolverMode::Both;
    }
    if (value == "svrp") {
        return SolverMode::Svrp;
    }
    if (value == "vrp") {
        return SolverMode::Vrp;
    }
    throw runtime_error("--solver must be one of: both, svrp, vrp");
}

ProgramOptions parse_options(int argc, char* argv[]) {
    ProgramOptions options;

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            throw runtime_error("");
        } else if (arg == "--graph") {
            options.graph_path = require_value(i, argc, argv, arg);
        } else if (arg == "--goals") {
            string value = require_value(i, argc, argv, arg);
            if (value.find(',') != string::npos || value.find(';') != string::npos) {
                options.fixed_goals = parse_int_list(value, "--goals");
                options.num_goals = (int)options.fixed_goals.size();
            } else {
                options.num_goals = parse_int_arg(value, arg);
            }
        } else if (arg == "--goal-ids") {
            options.fixed_goals = parse_int_list(require_value(i, argc, argv, arg), arg);
            options.num_goals = (int)options.fixed_goals.size();
        } else if (arg == "--goals-file") {
            options.fixed_goals = read_goal_ids_file(require_value(i, argc, argv, arg));
            options.num_goals = (int)options.fixed_goals.size();
        } else if (arg == "--tests") {
            options.num_tests = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--seed") {
            options.seed = parse_seed_arg(require_value(i, argc, argv, arg));
        } else if (arg == "--depot") {
            options.depot = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--solver") {
            options.solver = parse_solver(require_value(i, argc, argv, arg));
        } else if (arg == "--ants") {
            options.ants = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--iterations") {
            options.iterations = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--threads") {
            options.threads = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--battery") {
            options.battery = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--k-cover") {
            options.k_cover = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--output") {
            options.output_path = require_value(i, argc, argv, arg);
        } else if (!is_option(arg) && options.num_goals < 0) {
            options.num_goals = parse_int_arg(arg, "<num_goals>");
        } else if (!is_option(arg) && options.graph_path.empty()) {
            options.graph_path = arg;
        } else {
            throw runtime_error("Unknown or duplicate argument: " + arg);
        }
    }

    return options;
}

void print_usage(const char* program_name) {
    cerr << "Usage:\n";
    cerr << "  " << program_name << " --graph <csv> --goals <n> [options]\n";
    cerr << "  " << program_name << " <num_goals> <graph_csv> [options]\n\n";
    cerr << "Options:\n";
    cerr << "  --tests <n>           Number of random scenarios. Default: 10\n";
    cerr << "  --seed <n>            Seed for depot/goal sampling and VRP ACO. Default: random_device\n";
    cerr << "  --depot <id>          Fixed depot id. Default: random per test\n";
    cerr << "  --goal-ids <ids>      Fixed comma/space-separated goal ids\n";
    cerr << "  --goals-file <path>   File with comma/space-separated goal ids\n";
    cerr << "  --solver <mode>       both, svrp, or vrp. Default: both\n";
    cerr << "  --ants <n>            Ants per iteration. Default: 100\n";
    cerr << "  --iterations <n>      Stop after this many non-improving iterations. Default: 20\n";
    cerr << "  --threads <n>         OpenMP thread count. Default: 5\n";
    cerr << "  --battery <n>         Battery capacity. Default: 50000\n";
    cerr << "  --k-cover <n>         k for k-path-cover shrink graph. Default: 24\n";
    cerr << "  --output <path>       Write result lines to file instead of stdout\n\n";

    cerr << "Example:\n";
    cerr << "  " << program_name << " --graph art/u100/graph_000_n100_c0_u1_seed20062.csv --goals 10 --seed 42 --solver both\n";
}

void validate_positive(const ProgramOptions& options) {
    if (options.num_tests <= 0) {
        throw runtime_error("--tests must be positive");
    }
    if (options.ants <= 0) {
        throw runtime_error("--ants must be positive");
    }
    if (options.iterations <= 0) {
        throw runtime_error("--iterations must be positive");
    }
    if (options.threads <= 0) {
        throw runtime_error("--threads must be positive");
    }
    if (options.battery <= 0) {
        throw runtime_error("--battery must be positive");
    }
    if (options.k_cover <= 1) {
        throw runtime_error("--k-cover must be greater than 1");
    }
}

void validate_node_id(int id, int num_nodes, const string& name) {
    if (id < 0 || id >= num_nodes) {
        throw runtime_error(name + " id " + to_string(id) + " is outside graph node range 0.." + to_string(num_nodes - 1));
    }
}

set<int> build_fixed_goals(const vector<int>& goal_ids, int num_nodes) {
    set<int> goals;
    for (int goal: goal_ids) {
        validate_node_id(goal, num_nodes, "Goal");
        if (!goals.insert(goal).second) {
            throw runtime_error("Duplicate goal id: " + to_string(goal));
        }
    }
    return goals;
}

int choose_depot(mt19937& rng, int num_nodes, const set<int>& forbidden_goals) {
    uniform_int_distribution<int> node_distribution(0, num_nodes - 1);
    int depot = node_distribution(rng);
    while (forbidden_goals.find(depot) != forbidden_goals.end()) {
        depot = node_distribution(rng);
    }
    return depot;
}

set<int> choose_random_goals(mt19937& rng, int num_nodes, int num_goals, int depot) {
    uniform_int_distribution<int> node_distribution(0, num_nodes - 1);
    set<int> goals;
    while ((int)goals.size() < num_goals) {
        int new_goal = node_distribution(rng);
        if (new_goal != depot) {
            goals.insert(new_goal);
        }
    }
    return goals;
}

int main(int argc, char *argv[]) {
    ProgramOptions options;
    try {
        options = parse_options(argc, argv);
    } catch (const exception& e) {
        if (string(e.what()).empty()) {
            print_usage(argv[0]);
            return 0;
        }
        cerr << e.what() << endl;
        print_usage(argv[0]);
        return 1;
    }

    if (options.graph_path.empty() || options.num_goals < 0) {
        print_usage(argv[0]);
        return 1;
    }

    try {
        validate_positive(options);
    } catch (const exception& e) {
        cerr << e.what() << endl;
        return 1;
    }

    InputGraph input_graph;
    try {
        input_graph = read_graph(options.graph_path);
    } catch (const exception& e) {
        cerr << "Failed to read graph: " << e.what() << endl;
        return 1;
    }

    int num_nodes = input_graph.num_nodes;
    if (options.num_goals <= 0 || options.num_goals >= num_nodes) {
        cerr << "num_goals must be between 1 and " << num_nodes - 1 << endl;
        return 1;
    }

    set<int> fixed_goals;
    try {
        fixed_goals = build_fixed_goals(options.fixed_goals, num_nodes);
        if (options.depot.has_value()) {
            validate_node_id(*options.depot, num_nodes, "Depot");
            if (fixed_goals.find(*options.depot) != fixed_goals.end()) {
                throw runtime_error("Depot cannot also be a goal");
            }
        }
    } catch (const exception& e) {
        cerr << e.what() << endl;
        return 1;
    }

    if (!fixed_goals.empty() && (int)fixed_goals.size() != options.num_goals) {
        cerr << "Fixed goal count does not match --goals" << endl;
        return 1;
    }

    mt19937 rng;
    if (options.seed.has_value()) {
        rng.seed(*options.seed);
    } else {
        random_device rd;
        rng.seed(rd());
    }

    ofstream output_file;
    ostream* output = &cout;
    if (!options.output_path.empty()) {
        output_file.open(options.output_path);
        if (!output_file) {
            cerr << "Cannot open output file: " << options.output_path << endl;
            return 1;
        }
        output = &output_file;
    }

    for (int i = 0; i < options.num_tests; ++i) {
        set<int> goals = fixed_goals;
        int depot = options.depot.value_or(choose_depot(rng, num_nodes, goals));

        if (goals.empty()) {
            goals = choose_random_goals(rng, num_nodes, options.num_goals, depot);
        }
        unsigned int solver_seed = rng();

        Graph graph(num_nodes, input_graph.edge_points, input_graph.edge_parameters, depot, goals);

        string svrp_answer;
        string vrp_answer;
        if (options.solver == SolverMode::Both || options.solver == SolverMode::Svrp) {
            SVRPAntColony sant(graph, (int)goals.size(), options.ants, options.iterations,
                               options.threads, options.battery, options.k_cover, solver_seed);
            svrp_answer = sant.bicriterion_ant_svrp();
        }
        if (options.solver == SolverMode::Both || options.solver == SolverMode::Vrp) {
            VRPAntColony ant(graph, (int)goals.size(), options.ants, options.iterations,
                             options.threads, options.battery, options.k_cover, solver_seed);
            vrp_answer = ant.bicriterion_ant_vrp();
        }

        if (options.solver == SolverMode::Svrp) {
            *output << svrp_answer << endl;
        } else if (options.solver == SolverMode::Vrp) {
            *output << vrp_answer << endl;
        } else {
            *output << svrp_answer << "|" << vrp_answer << endl;
        }
    }

    return 0;
}
