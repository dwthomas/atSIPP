#include <unordered_set>
#include "plrtosipphonly.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include "newatsippgraph.hpp"
#include "sippgraph.hpp"
#include "hybrid.hpp"

std::vector<const SIPPState<Location> *> hybrid::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<const SIPPState<Location> *> path;
    m.init();
    m.search_timer.start();
    auto cur = source;
    double t = start_time;
    while(cur->configuration != dest){
        //search 
        //std::cerr << *cur << " at " << t << " ";
        // run NLASIPP
        path.emplace_back(cur);
        asipp::Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), plrtosipphonly::get_h(*cur, t, dest) , cur, nullptr, nullptr);
        asipp::search_core(g, open_list, dest, m, budget, plrtosipphonly::get_h);
        if(open_list.empty()){
            std::cerr << "No path found\n";
            exit(-1);
        }
        // Done NLASIPP
        //learn
        // static
        //asipp::dump_open(open_list);
        plrtosipphonly::lsslrtsipp(g, open_list, dest, m);
        //rtasipp learning
        double h_s_prime = std::numeric_limits<double>::infinity();
        for (const auto& node: open_list.queue){
            double h_s_nx = plrtosipphonly::get_h_s(node.state->configuration, dest);
            h_s_prime = std::min(h_s_prime, node.g.delta + h_s_nx);
            EdgeATF eatf = node.g;
            eatf.delta += h_s_nx;
            plrtosipphonly::add_h_dyn(*cur, eatf);
            //std::cerr << "n: " << node << "\n";
        }
        if(h_s_prime > plrtosipphonly::get_h_s(cur->configuration, dest)){
            plrtosipphonly::set_h_s(cur->configuration, h_s_prime);
        }

        // dynamic
        //dump_h_s(h_static);
        // commit 
        auto e = open_list.top().tla;
        t = e->duration.arrival_time(t);
        cur = e->destination;
    } 
    path.emplace_back(cur);
    m.search_timer.stop();
    return path;
}