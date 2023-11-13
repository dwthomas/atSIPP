#include "sipp.hpp"
#include "graph.hpp"
#include "structs.hpp"
#include <algorithm>

using namespace sipp;

bool isGoal(const Node& n, const Location& goal_loc){
    return n.node->state.loc == goal_loc;
}

void expand(const Node& cur, Open& open_list, const Location& goal_loc){
    for(GraphEdge * successor: cur.node->successors){
        double arrival_time = successor->edge.arrival_time(cur.g);
        if(open_list.expanded.contains(successor->destination)){
            continue;
        }
        else if (open_list.handles.contains(successor->destination)){
            auto handle = open_list.handles[successor->destination];
            if(arrival_time < (*handle).g){
                double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                open_list.decrease_key(handle ,arrival_time, h, successor->destination, successor->source);
            }
        }
        else{
            double h = eightWayDistance(successor->destination->state.loc, goal_loc);
            open_list.emplace(arrival_time, h, successor->destination, successor->source);
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

std::vector<GraphNode *> sipp::search(GraphNode * source, const Location& dest){
    Open open_list;
    open_list.emplace(0.0, eightWayDistance(dest, source->state.loc), source, nullptr);
    while(!open_list.empty()){
        //dump_open(open_list);
        Node cur = open_list.top();
        //std::cout << *cur.node << "\n";
        if(isGoal(cur, dest)){
            return backup(cur, open_list);
        }
        open_list.pop();
        expand(cur, open_list, dest);
    }
    std::cerr << "Failed to find path\n";
    exit(-1);
}