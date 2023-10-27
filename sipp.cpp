#include "sipp.hpp"
#include "structs.hpp"

using namespace sipp;

bool isGoal(const Node& n, const Location& goal_loc){
    return n.node->state.loc == goal_loc;
}

void expand(const Node& cur, Open_t& open_list, const Location& goal_loc){
    for(GraphEdge * successor: cur.node->successors){
        double arrival_time = successor->edge.arrival_time(cur.g);
        if(arrival_time < successor->destination->earliest_arrival){
            successor->destination->earliest_arrival = arrival_time;
            double h = eightWayDistance(successor->destination->state.loc, goal_loc);
            open_list.emplace(arrival_time, arrival_time + h, successor->destination, successor);
        }
    }
}

Node sipp::search(GraphNode * source, const Location& dest){
    Open_t open_list;
    open_list.emplace(0.0, eightWayDistance(dest, source->state.loc), source, nullptr);
    while(!open_list.empty()){
        Node cur = open_list.top();
        if(isGoal(cur, dest)){
            return cur;
        }
        open_list.pop();
        expand(cur, open_list, dest);
    }
    std::cerr << "Failed to find path\n";
    exit(-1);
}