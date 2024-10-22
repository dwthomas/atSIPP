#include "repeat.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include <time.h>

double update_reference_time(const EdgeATF& path, asipp::Open& open_list){
    double upper_bound = path.beta;
    double lower_bound = path.alpha;
    //std::cerr << "Updating reference time:\n";
    //std::cerr << path << "\nOpen:\n";
    //asipp::dump_open(open_list);
    while(lower_bound < upper_bound){
        if(open_list.empty()){
            return std::numeric_limits<double>::infinity();
        }
        auto n = open_list.top();
        open_list.pop();
        while(!open_list.empty() && n.f == path.earliest_arrival_time()){
            auto n = open_list.top();
            open_list.pop();
        }
        lower_bound = n.f - path.delta;
        if (n.g.alpha > lower_bound){
            return lower_bound;
        } 
    }
    return upper_bound;
}

CompoundATF<std::vector<const SIPPState<Location> *>> rePEAT::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time){
    double t_ref = start_time;
    std::vector<const SIPPState<Location> *> path;
    CompoundATF<std::vector<const SIPPState<Location> *>> solutions;
    m.init();
    struct timespec ts1, ts2;
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    while(t_ref < end(source->safe_interval)){
        //std::cerr << "tref: " << t_ref << "\n";
        asipp::Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t_ref, std::numeric_limits<double>::infinity(), 0.0), eightWayDistance(dest, source->configuration), source, nullptr, nullptr);
        auto res = asipp::search_core(g, open_list, dest, m);
        solutions.insert(res.second, res.first);
        t_ref = update_reference_time(res.second, open_list);
    }
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    m.search_time = 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
    return solutions;
}