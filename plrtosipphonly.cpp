#include <unordered_set>
#include "plrtosipphonly.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

std::unordered_map<Location, double> plrtosipphonly::h_static;
std::unordered_map<const GraphNode *, plrtosipphonly::CATF> plrtosipphonly::h_dynamic;

inline void dump_h(){
    std::cerr << "h_static\n";
    for(auto acc: plrtosipphonly::h_static){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
}

void plrtosipphonly::lsslrtsipp(const Open& open_list, const Location& dest){
    std::unordered_map<Location, double> h_s_prime;
    auto closed = open_list.expanded; 
    LSSOpen dijkstraOpen;
    for(auto n: open_list.queue){
        auto h_s = get_h_s(*n.node, dest);
        h_s_prime[n.node->state.loc] = h_s;
        dijkstraOpen.emplace(h_s, n.node);
    }
    std::cerr << "Dijkstra open:\n";
    dump_open(dijkstraOpen);
    while(!dijkstraOpen.empty() && !closed.empty()){
        auto n = dijkstraOpen.top();
        //std::cerr << "Dijkstra n: " << n << "\n";
        dijkstraOpen.pop();
        closed.erase(n.node);
        auto n_loc = n.node->state.loc;
        double h_s_n = h_s_prime[n_loc];
        for (auto p: n.node->predecessors){
            if(closed.find(p->source) == closed.end()){
                continue;
            }
            auto p_loc = p->source->state.loc;
            auto i = h_s_prime.find(p_loc);
            if(i == h_s_prime.end()){
                h_s_prime[p_loc] = std::numeric_limits<double>::infinity();
            }
            double h_s_p = h_s_prime[p_loc];
            double edge_cost =  eightWayDistance(p_loc, n_loc);
            if(h_s_p > edge_cost + h_s_n){
                //std::cerr << "learning: " <<  edge_cost + h_s_n << " to " << p_loc << " from " << n_loc << "\n";
                h_s_prime[p_loc] = edge_cost + h_s_n;
                dijkstraOpen.emplace(edge_cost + h_s_n, p->source);
            }
        }
    }
    for(auto x: h_s_prime){
        set_h_s(x.first, std::max(get_h_s(x.first, dest), x.second));
    }
}



std::vector<GraphNode *> plrtosipphonly::search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<GraphNode *> path;
    m.init();
    auto cur = source;
    double t = start_time;
    while(!isGoal(*cur, dest)){
        //search 
        std::cerr << *cur << " at " << t << " ";
        m.search_timer.resume();
        // run NLASIPP
        path.emplace_back(cur);
        Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), get_h(*cur, t, dest) , cur, nullptr);
        auto res = asipp::search_core(open_list, dest, m, budget, get_h);
        if(open_list.empty()){
            std::cerr << "No path found\n";
            exit(-1);
        }
        // Done NLASIPP
        m.search_timer.stop();
        //learn
        // static
        //asipp::dump_open(open_list);
        lsslrtsipp(open_list, dest);
        // dynamic
        dump_h_s(h_static);
        // commit 
        GraphNode * best_successor = nullptr;
        double best_f = std::numeric_limits<double>::infinity();
        double best_g;
        std::cerr << "options:\n";
        for(const auto& e: cur->successors){
            double g = e->edge.arrival_time(t);
            double f = g + get_h(*e->destination, g, dest);
            std::cerr << *e->destination << " at g:" << g << " f: " << f << "\n";  
            if (f < best_f){
                best_f = f;
                best_g = g;
                best_successor = e->destination;
            }
        }
        std::cerr << "\n";
        if (best_successor == nullptr){
            std::cerr << "No successor found!\n";
            exit(-1);
        }
        cur = best_successor;
        t = best_g;
    } 
    path.emplace_back(cur);
    return path;
}