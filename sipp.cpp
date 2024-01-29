#include "sipp.hpp"
#include "atsippgraph.hpp"
#include "structs.hpp"
#include <algorithm>
#include <cmath>

using namespace sipp;

bool isGoal(const Node& n, const Location& goal_loc){
    return n.node->state.loc == goal_loc;
}

void expand(const Node& cur, Open& open_list, const Location& goal_loc, MetaData & m){
    m.expanded++;
    for(GraphEdge * successor: cur.node->successors){
        double arrival_time = successor->edge.arrival_time(cur.g);
        if(cur.g >= successor->edge.beta || end(cur.node->state.interval) <= successor->edge.zeta){
            continue;
        } 
        if(open_list.expanded.contains(successor->destination)){
            continue;
        }
        else if (open_list.handles.contains(successor->destination)){
            auto handle = open_list.handles[successor->destination];
            if(arrival_time < (*handle).g){
                m.decreased++;
                //std::cerr << "Decrease " << *handle << "\n";
                double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                open_list.decrease_key(handle, arrival_time, h, successor->destination, successor->source);
            }
        }
        else{
            m.generated++;
            double h = eightWayDistance(successor->destination->state.loc, goal_loc);
            open_list.emplace(arrival_time, h, successor->destination, successor->source);
            //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
        }
    }
}

void dump_open(const Open& open_list){
    Queue::ordered_iterator cur = open_list.queue.ordered_begin();
    Queue::ordered_iterator end = open_list.queue.ordered_end();
    std::cerr << "Open:";
    while(cur != end){
        std::cerr << "\t" << *cur << "\n";
        cur = std::next(cur);
    }
}

std::vector<AtsippGraphNode *> backup(const Node& n, Open& open_list){
    std::vector<AtsippGraphNode *> res;
    AtsippGraphNode* cur = n.node;
    while(cur != nullptr){
        res.push_back(cur);
        cur = open_list.parent[cur];
    }
    std::reverse(res.begin(), res.end());
    std::cout << "Arrival time: " << n.f << "\n";
    return res;
}

std::vector<AtsippGraphNode *> sipp::search(AtsippGraphNode * source, const Location& dest, MetaData& m, double start_time){
    Open open_list;
    m.init();
    m.search_timer.start();
    open_list.emplace(start_time, eightWayDistance(dest, source->state.loc), source, nullptr);
    while(!open_list.empty()){
        //dump_open(open_list);
        Node cur = open_list.top();
        //std::cout << *cur.node << "\n";
        if(isGoal(cur, dest)){
            m.search_timer.stop();
            return backup(cur, open_list);
        }
        open_list.pop();
        expand(cur, open_list, dest, m);
    }
    std::cerr << "Failed to find path\n";
    exit(-1);
}