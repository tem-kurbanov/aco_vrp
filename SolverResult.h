#ifndef ACO_VRP_SOLVER_RESULT_H
#define ACO_VRP_SOLVER_RESULT_H

#include <sstream>
#include <string>
#include <vector>

struct SolverParetoPoint {
    std::vector<int> objectives;
    int route_count = 0;
};

struct SolverRunResult {
    // A completed process can still report disconnected/no_solution; inspect status.
    std::string solver;
    std::string status = "error";
    std::string message;
    int graph_nodes = 0;
    int graph_edges = 0;
    int iterations = 0;
    int invalid_candidates = 0;
    double preprocessing_seconds = 0.0;
    double optimization_seconds = 0.0;
    double total_seconds = 0.0;
    std::vector<SolverParetoPoint> pareto;

    std::string legacy_answer() const {
        if (status != "ok") {
            return {};
        }

        std::ostringstream stream;
        stream << graph_edges << "," << iterations << "," << optimization_seconds << " ";
        for (const auto& point: pareto) {
            if (point.objectives.size() >= 2) {
                stream << "(" << point.objectives[0] << ";" << point.objectives[1] << "),";
            }
        }
        return stream.str();
    }
};

#endif
