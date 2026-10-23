

#ifndef ACO_VRP_LABEL_H
#define ACO_VRP_LABEL_H

#include <memory>
#include <utility>
#include <vector>

using namespace std;

struct Label;
using LabelPtr = shared_ptr<Label>;

struct Label{
    int node;
    vector<int> parameters;
    LabelPtr parent;
    // Cover search: index within a cover-arc bundle; route planner: edge ID.
    int used_edge;

    Label(int n, vector<int> params, LabelPtr p, int edge) : node(n), parameters(std::move(params)), parent(std::move(p)), used_edge(edge) {}

};

struct lex_compare{
    bool operator() (const LabelPtr& lhs, const LabelPtr& rhs) const {
        for (int i = 0; i < (int)lhs->parameters.size(); ++i) {
            if (lhs->parameters[i] < rhs->parameters[i])
                return true;
            else if (lhs->parameters[i] > rhs->parameters[i])
                return false;
        }

        return lhs->node < rhs->node;
    }
};

struct soc_lex_compare{
    // Backward energy summaries minimize required/net charge, maximize ending charge.
    bool operator() (const LabelPtr& lhs, const LabelPtr& rhs) const {
        for (int i = 0; i < (int)lhs->parameters.size() - 1; ++i) {
            if (lhs->parameters[i] < rhs->parameters[i])
                return true;
            else if (lhs->parameters[i] > rhs->parameters[i])
                return false;
        }

        if (lhs->parameters[(int)lhs->parameters.size() - 1] > rhs->parameters[(int)lhs->parameters.size() - 1])
            return true;
        else if (lhs->parameters[(int)lhs->parameters.size() - 1] < rhs->parameters[(int)lhs->parameters.size() - 1])
            return false;

        return lhs->node < rhs->node;
    }
};

#endif //ACO_VRP_LABEL_H
