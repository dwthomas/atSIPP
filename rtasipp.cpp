#include <unordered_set>
#include "rtasipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

std::unordered_map<Location, double> rtasipp::h_static;
std::unordered_map<const GraphNode *, rtasipp::CATF> rtasipp::h_dynamic;

inline void dump_h(){
    std::cerr << "h_static\n";
    for(auto acc: rtasipp::h_static){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
}

bool isGoal(const rtasipp::Node& n, const Location& goal_loc){
    return n.node->state.loc == goal_loc;
}

void expand(const rtasipp::Node& cur, rtasipp::Open& open_list, const Location& goal_loc, MetaData & m, double (*hf)(const GraphNode&, double , const Location& ) = asipp::h_eight_way_helper){
    m.expanded++;
    double zeta = cur.g.zeta;
    for(GraphEdge * successor: cur.node->successors){
        auto tla = cur.tla;
        if (tla == nullptr){
            tla = successor;
        }
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
                double h = hf(*successor->destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                open_list.decrease_key(handle ,arrival_time_function, h, successor->destination, successor->source, tla);
            }
        }
        else{
            m.generated++;
            double h = hf(*successor->destination, arrival_time_function.earliest_arrival_time(), goal_loc);
            //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
            open_list.emplace(arrival_time_function, h, successor->destination, successor->source, tla);
            //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
        }
    }
}


void search_core(rtasipp::Open& open_list, const Location& dest, MetaData & m, long expansion_budget = -1, double (*hf)(const GraphNode&, double , const Location& ) = asipp::h_eight_way_helper){
    long start_expansions = m.expanded;
    while(!open_list.empty()){
        //dump_open(open_list);
        auto cur = open_list.top();
        //std::cout << *cur.node << "\n";
        if(isGoal(cur, dest) || (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget)){
            return;
        }
        open_list.pop();
        expand(cur, open_list, dest, m, hf);
    }
    std::cerr << "Failed to find path\n";
    exit(-1);
}

std::vector<GraphNode *> rtasipp::search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<GraphNode *> path;
    m.init();
    m.search_timer.start();
    auto cur = source;
    double t = start_time;
    while(!isGoal(*cur, dest)){
        //search 
        //std::cerr << "cur: " << *cur << " at " << t << " ";
        // run NLASIPP
        path.emplace_back(cur);
        Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), get_h(*cur, t, dest) , cur, nullptr, nullptr);
        search_core(open_list, dest, m, budget, get_h);
        // Done NLASIPP
        //learn
        double h_s_prime = std::numeric_limits<double>::infinity();
        for (auto node: open_list.queue){
            double h_s_nx = get_h_s(*node.node, dest);
            h_s_prime = std::min(h_s_prime, node.g.delta + h_s_nx);
            EdgeATF eatf = node.g;
            eatf.delta += h_s_nx;
            add_h_dyn(*cur, eatf);
        }
        if(h_s_prime > get_h_s(*cur, dest)){
            set_h_s(cur->state.loc, h_s_prime);
        }
        //dump_h_s(h_static);
        auto e = open_list.top().tla;
        t = e->edge.arrival_time(t);
        cur = e->destination;
        // GraphNode * best_successor = nullptr;
        // GraphEdge * best_edge;
        // double best_f = std::numeric_limits<double>::infinity();
        // double best_g;
        // std::cerr << "options:\n";
        // for(const auto& e: cur->successors){
        //     double g = e->edge.arrival_time(t);
        //     std::cerr << *e << " at " << t << " = " << g << "\n";
        //     double f = g + get_h(*e->destination, g, dest);
        //     std::cerr << *e->destination << " at g:" << g << " f: " << f << "\n";  
        //     if (f < best_f){
        //         best_f = f;
        //         best_g = g;
        //         best_successor = e->destination;
        //         best_edge = e;
        //     }
        // }
        // std::cerr << "chose: " <<  *best_edge << " at g:" << best_g << " f: " << best_f  << "\n";
        // if (best_successor == nullptr){
        //     std::cerr << "No successor found!\n";
        //     exit(-1);
        // }
        // cur = best_successor;
        // t = best_g;
        // auto n = open_list.top().node;
        // while(open_list.parent[n] != nullptr){
        //     auto nn = open_list.parent[n];
        //     if (open_list.parent[nn] == nullptr){
        //         for (auto succ: cur->successors){
        //             if(succ->destination == n){
        //                 t = succ->edge.arrival_time(t);
        //                 break;
        //             }
        //         }
        //         cur = n; //best TLA
        //         //std::cerr << "best TLA:" << *n << "\n";
        //         break;
        //     }
        //     n = nn;
        // }
    } 
    path.emplace_back(cur);
    m.search_timer.stop();
    return path;
}