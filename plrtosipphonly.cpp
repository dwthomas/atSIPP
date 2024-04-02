#include <unordered_set>
#include "plrtosipphonly.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include "rtasipp.hpp"
#include "newatsippgraph.hpp"
#include "sippgraph.hpp"

//std::unordered_map<Location, double> plrtosipphonly::h_static;
//std::unordered_map<const SIPPState<Location> *, plrtosipphonly::CATF> plrtosipphonly::h_dynamic;

// inline void dump_h(){
//     std::cerr << "h_static\n";
//     for(auto acc: plrtosipphonly::h_static){
//         std::cerr << acc.first << " " << acc.second << "\n";
//     } 
//     std::cerr << "h_static done\n";
// }

// bool isGoal(const plrtosipphonly::Node& n, const Location& goal_loc){
//     return n.node->state.loc == goal_loc;
// }

// void expand(const plrtosipphonly::Node& cur, plrtosipphonly::Open& open_list, const Location& goal_loc, MetaData & m, double (*hf)(const AtsippGraphNode&, double , const Location& ) = asipp::h_eight_way_helper){
//     m.expanded++;
//     double zeta = cur.g.zeta;
//     for(GraphEdge * successor: cur.node->successors){
//         auto tla = cur.tla;
//         if (tla == nullptr){
//             tla = successor;
//         }
//         if(cur.g.earliest_arrival_time() >= successor->edge.beta || cur.g.supremum_arrival_time() <= successor->edge.zeta){
//             continue;
//         } 
//         double alpha = std::max(cur.g.alpha, successor->edge.alpha - cur.g.delta);
//         double beta = std::min(cur.g.beta, successor->edge.beta - cur.g.delta);
//         double delta = successor->edge.delta + cur.g.delta;
//         EdgeATF arrival_time_function(zeta, alpha, beta, delta);
//         if(open_list.expanded.contains(successor->destination)){
//             continue;
//         }
//         else if (open_list.handles.contains(successor->destination)){
//             auto handle = open_list.handles[successor->destination];
//             if(arrival_time_function.earliest_arrival_time() < (*handle).g.earliest_arrival_time()){
//                 m.decreased++;
//                 double h = hf(*successor->destination, arrival_time_function.earliest_arrival_time(), goal_loc);
//                 //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
//                 open_list.decrease_key(handle ,arrival_time_function, h, successor->destination, successor->source, tla);
//             }
//         }
//         else{
//             m.generated++;
//             double h = hf(*successor->destination, arrival_time_function.earliest_arrival_time(), goal_loc);
//             //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
//             open_list.emplace(arrival_time_function, h, successor->destination, successor->source, tla);
//             //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
//         }
//     }
// }


// void search_core(plrtosipphonly::Open& open_list, const Location& dest, MetaData & m, long expansion_budget = -1, double (*hf)(const AtsippGraphNode&, double , const Location& ) = asipp::h_eight_way_helper){
//     long start_expansions = m.expanded;
//     while(!open_list.empty()){
//         //dump_open(open_list);
//         auto cur = open_list.top();
//         //std::cout << *cur.node << "\n";
//         if(isGoal(cur, dest) || (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget)){
//             return;
//         }
//         open_list.pop();
//         expand(cur, open_list, dest, m, hf);
//     }
//     std::cerr << "Failed to find path\n";
//     exit(-1);
// }

void plrtosipphonly::lsslrtsipp(const AtSippGraph<Location>& g, const asipp::Open& open_list, const Location& dest, MetaData& m){
    std::unordered_map<Location, double> h_s_prime;
    auto closed = open_list.expanded; 
    LSSOpen dijkstraOpen;
    for(auto n: open_list.queue){
        auto h_s = rtasipp::get_h_s(n.state, dest);
        h_s_prime[n.state->configuration] = h_s;
        dijkstraOpen.emplace(h_s, n.state);
    }
    //std::cerr << "Dijkstra open:\n";
    //dump_open(dijkstraOpen);
    while(!dijkstraOpen.empty() && !closed.empty()){
        auto n = dijkstraOpen.top();
        m.learn_expanded++;
        //std::cerr << "Dijkstra n: " << n << "\n";
        dijkstraOpen.pop();
        closed.erase(n.node);
        auto n_loc = n.node->configuration;
        double h_s_n = h_s_prime[n_loc];
        for (const auto& p: g.predecessors.at(n.node)){
            if(closed.find(p.source) == closed.end()){
                continue;
            }
            auto p_loc = p.source->configuration;
            auto i = h_s_prime.find(p_loc);
            if(i == h_s_prime.end()){
                h_s_prime[p_loc] = std::numeric_limits<double>::infinity();
            }
            double h_s_p = h_s_prime[p_loc];
            double edge_cost =  eightWayDistance(p_loc, n_loc);
            if(h_s_p > edge_cost + h_s_n){
                //std::cerr << "learning: " <<  edge_cost + h_s_n << " to " << p_loc << " from " << n_loc << "\n";
                h_s_prime[p_loc] = edge_cost + h_s_n;
                dijkstraOpen.emplace(edge_cost + h_s_n, p.source);
            }
        }
    }
    for(auto x: h_s_prime){
        rtasipp::set_h_s(x.first, std::max(rtasipp::get_h_s(x.first, dest), x.second));
    }
} 



std::vector<const SIPPState<Location> *> plrtosipphonly::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<const SIPPState<Location> *> path;
    struct timespec ts1, ts2;
    m.init();
    auto cur = source;
    double t = start_time;
    while(cur->configuration != dest){
        //search 
        //std::cerr << *cur << " at " << t << " ";
        // run NLASIPP
        path.emplace_back(cur);
        asipp::Open open_list;
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), rtasipp::get_h(*cur, t, dest) , cur, nullptr, nullptr);
        asipp::search_core(g, open_list, dest, m, budget, rtasipp::get_h);
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        m.search_time += 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
        if(open_list.empty()){
            std::cerr << "No path found\n";
            exit(-1);
        }
        // Done NLASIPP
        //learn
        // static
        //asipp::dump_open(open_list);
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        lsslrtsipp(g, open_list, dest, m);
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        m.learning_time += 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
        // dynamic
        //dump_h_s(h_static);
        // commit 
        auto e = open_list.top().tla;
        t = e->duration.arrival_time(t);
        cur = e->destination;
    } 
    path.emplace_back(cur);
    std::cout << "Arrival time: " << t << "\n";
    return path;
}