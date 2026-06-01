

#ifndef ACO_VRP_LABEL_H
#define ACO_VRP_LABEL_H

#include <vector>

using namespace std;

struct Label{
    int node;
    vector<int> parameters;
    Label *parent;
    int used_edge;

    Label(int n, vector<int> params, Label *p, int edge) : node(n), parameters(move(params)), parent(p), used_edge(edge) {}

};

struct lex_compare{
    bool operator() (Label* lhs, Label* rhs) const {
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
    bool operator() (Label* lhs, Label* rhs) const {
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
