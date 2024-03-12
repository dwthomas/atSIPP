#include "sipp.hpp"
#include "graph.hpp"
#include "structs.hpp"
#include <algorithm>
#include <cmath>

using namespace sipp;

bool isGoal(const Node& n, const Location& goal_loc){
    return n.node->state.loc == goal_loc;
}

void expand(const Node& cur, Open& open_list, const Location& goal_loc, MetaData & m){
    m.expanded++;
    (void)goal_loc; // reserved for heuristic 
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
                std::cerr << "Decrease " << *handle << "\n";
                double h = 0;
                open_list.decrease_key(handle, arrival_time, h, successor->destination, successor->source);
            }
        }
        else{
            m.generated++;
            double h = 0;
            open_list.emplace(arrival_time, h, successor->destination, successor->source);
            //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
        }
    }
}

   template<typename Open_t>
    inline void dump_open(const Open_t& open_list){
        auto cur = open_list.queue.ordered_begin();
        auto end = open_list.queue.ordered_end();
        std::cerr << "Open:";
        while(cur != end){
            std::cerr << "\t" << *cur << "\n";
            cur = std::next(cur);
        }
    }

std::vector<GraphNode *> backup(const Node& n, Open& open_list){
    //auto lookup_start_time = std::chrono::high_resolution_clock::now();
    std::vector<GraphNode *> res;
    GraphNode* cur = n.node;
    while(cur != nullptr){
        res.push_back(cur);
        cur = open_list.parent[cur];
    }
    std::reverse(res.begin(), res.end());
    //auto lookup_time = std::chrono::high_resolution_clock::now();
    //auto lookup_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(lookup_time - lookup_start_time);
    //std::cout << "Lookup time: " << lookup_duration.count() << " nanoseconds\n";
    //std::cout << "Arrival time: " << n.f << "\n";
    return res;
}

std::vector<GraphNode *> sipp::search(GraphNode * source, const Location& dest, MetaData& m, double start_time){
    Open open_list;
    m.init();
    open_list.emplace(start_time, 0, source, nullptr);
    while(!open_list.empty()){
        dump_open(open_list);
        Node cur = open_list.top();
        //std::cout << "Current " << cur << " " << open_list.queue.size() << "\n";
        if(isGoal(cur, dest)){
            return backup(cur, open_list);
        }
        open_list.pop();
        expand(cur, open_list, dest, m);
    }
    std::cerr << "Failed to find path\n";
    exit(-1);
}