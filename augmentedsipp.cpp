#include "augmentedsipp.hpp"
#include "graph.hpp"
#include "structs.hpp"
#include <algorithm>
#include <limits>
#include <utility>

using namespace asipp;

bool isGoal(const Node& n, const Location& goal_loc){
    return n.node->state.loc == goal_loc;
}

void expand(const Node& cur, Open& open_list, const Location& goal_loc, MetaData & m){
    (void)goal_loc; //reserved for future heuristic work
    m.expanded++;
    double zeta = cur.g.zeta;
    for(GraphEdge * successor: cur.node->successors){
        if(cur.g.earliest_arrival_time() >= successor->edge.beta || cur.g.supremum_arrival_time() <= successor->edge.zeta){
            continue;
        } 
        double alpha = std::max(cur.g.alpha, successor->edge.alpha - cur.g.delta);
        double beta = std::min(cur.g.beta, successor->edge.beta - cur.g.delta);
        double delta = successor->edge.delta + cur.g.delta;
        EdgeATF arrival_time_function(zeta, alpha, beta, delta);
        if(open_list.expanded.contains(successor->destination)){
            continue;
        }
        else if (open_list.handles.contains(successor->destination)){
            auto handle = open_list.handles[successor->destination];
            if(arrival_time_function.earliest_arrival_time() < (*handle).g.earliest_arrival_time()){
                m.decreased++;
                double h = 0;
                open_list.decrease_key(handle, arrival_time_function, h, successor->destination, successor->source);
            }
        }
        else{
            m.generated++;
            double h = 0;
            open_list.emplace(arrival_time_function, h, successor->destination, successor->source);
        }
    }
}

void dump_open(const Open& open_list){
    Queue open_list_copy(open_list.queue);
    std::cerr << "Open:";
    while(!open_list_copy.empty()){
        std::cerr << "\t" << open_list_copy.top() << "\n";
        open_list_copy.pop();
    }
}

std::vector<GraphNode *> backup(const Node& n, Open& open_list){
    std::vector<GraphNode *> res;
    GraphNode* cur = n.node;
    while(cur != nullptr){
        res.push_back(cur);
        cur = open_list.parent[cur];
    }
    std::reverse(res.begin(), res.end());
    return res;
}

std::pair<std::vector<GraphNode *>, EdgeATF> asipp::search(GraphNode * source, const Location& dest, MetaData & m, double start_time){
    Open open_list;
    m.init();
    open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), start_time, std::numeric_limits<double>::infinity(), 0.0), 0, source, nullptr);
    while(!open_list.empty()){
        //dump_open(open_list);
        Node cur = open_list.top();
        std::cout << "Current " << cur << "\n";
        if(isGoal(cur, dest)){
            return std::make_pair(backup(cur, open_list), cur.g);
        }
        open_list.pop();
        expand(cur, open_list, dest, m);
    }
    std::cerr << "Failed to find path\n";
    exit(-1);
}