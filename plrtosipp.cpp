#include <unordered_set>
#include "plrtosipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

std::unordered_map<Location, double> plrtosipp::h_static;
std::unordered_map<const GraphNode *, plrtosipp::CATF> plrtosipp::h_dynamic;

inline void dump_h(){
    std::cerr << "h_static\n";
    for(auto acc: plrtosipp::h_static){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
}


bool isGoal(const plrtosipp::Node& n, const Location& goal_loc){
    return n.node->state.loc == goal_loc;
}

void expand(const plrtosipp::Node& cur, plrtosipp::Open& open_list, const Location& goal_loc, MetaData & m, double (*hf)(const GraphNode&, double , const Location& ) = asipp::h_eight_way_helper){
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


void search_core(plrtosipp::Open& open_list, const Location& dest, MetaData & m, long expansion_budget = -1, double (*hf)(const GraphNode&, double , const Location& ) = asipp::h_eight_way_helper){
    long start_expansions = m.expanded;
    asipp::dump_open(open_list);
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

void plrtosipp::lsslrtsipp(const Open& open_list, const Location& dest){
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

void plrtosipp::plrtolearn(const Open& open_list, const Location& dest){
    auto closed = open_list.expanded; 
    for (const auto& s: closed){ // node in closed
        h_dynamic[s.first] = CATF();   
    }
    DijkstraOpen dijkstraOpen;
    for (const auto& s: open_list.queue){
        dijkstraOpen.emplace(s.g, s.node);
        h_dynamic[s.node] = CATF();
        h_dynamic[s.node].insert(shiftIdentity(get_h_s(s.node->state.loc, dest)), nullptr);
    }
    while(!dijkstraOpen.empty() && !closed.empty()){
        auto n = dijkstraOpen.top();
        dijkstraOpen.pop();
        closed.erase(n.node);
        auto h_d_n = h_dynamic[n.node];
        for (auto e: n.node->predecessors){
            if(closed.find(e->source) == closed.end()){
                continue;
            }
            for (const auto& ap: h_d_n.edges()){
                const auto& ae = e->edge;
                auto a_p_prime = compose(ap, ae);
                add_h_dyn(*e->source, a_p_prime);
                dijkstraOpen.emplace(a_p_prime, e->source);
            }
        }
    }
}

std::vector<GraphNode *> plrtosipp::search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<GraphNode *> path;
    m.init();
    auto cur = source;
    double t = start_time;
    while(!isGoal(*cur, dest)){
        //search 
        //std::cerr << *cur << " at " << t << " ";
        m.search_timer.resume();
        // run NLASIPP
        path.emplace_back(cur);
        Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), get_h(*cur, t, dest) , cur, nullptr, nullptr);
        search_core(open_list, dest, m, budget, get_h);
        // Done NLASIPP
        m.search_timer.stop();
        //learn
        // static
        lsslrtsipp(open_list, dest);
        // dynamic
        plrtolearn(open_list, dest);
        // commit 
        auto e = open_list.top().tla;
        t = e->edge.arrival_time(t);
        cur = e->destination;
    } 
    path.emplace_back(cur);
    return path;
}