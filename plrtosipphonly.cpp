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
    while(!dijkstraOpen.empty() && !closed.empty()){
        auto n = dijkstraOpen.top();
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
        dump_h_s(h_static);
        m.search_timer.resume();
        // run NLASIPP
        path.emplace_back(cur);
        Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), get_h(*cur, t, dest) , cur, nullptr);
        asipp::search_core_noprune(open_list, dest, m, budget, get_h);
        if(open_list.empty()){
            std::cerr << "No path found\n";
            exit(-1);
        }
        // Done NLASIPP
        m.search_timer.stop();
        //learn
        // static
        lsslrtsipp(open_list, dest);
        // dynamic
        asipp::dump_open(open_list);
        // commit 
        if (open_list.queue.size() < 1){
            //asipp::dump_open(open_list);
            std::cout << "No path for agent!\n";
            exit(-1); 
        }
        auto n = open_list.top().node;
        while(open_list.parent[n] != nullptr){
            auto nn = open_list.parent[n];
            if (open_list.parent[nn] == nullptr){
                for (auto succ: cur->successors){
                    if(succ->destination == n){
                        t = succ->edge.arrival_time(t);
                        break;
                    }
                }
                cur = n; //best TLA
                //std::cerr << "best TLA:" << *n << "\n";
                break;
            }
            n = nn;
        }
    } 
    return path;
}