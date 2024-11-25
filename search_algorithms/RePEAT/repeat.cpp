#include "repeat.hpp"
#include "data_structures/atf.hpp"
#include <time.h>

double update_reference_time(const EdgeATF& path, asipp::Open<rePEAT::AtSIPPNodeComp>& open_list){
    double upper_bound = path.beta;
    double lower_bound = path.alpha;
    // std::cerr << "Updating reference time:\n";
    // std::cerr << lower_bound <<  " " << upper_bound << "\n";
    //std::cerr << path << "\n";
    //asipp::dump_open(open_list);
    while(lower_bound < upper_bound){
        if(open_list.empty()){
            return std::numeric_limits<double>::infinity();
        }
        auto n = open_list.top();
        open_list.pop();
        // while(!open_list.empty() && n.f == path.earliest_arrival_time()){
        //     auto n = open_list.top();
        //     open_list.pop();
        // }
        lower_bound = n.f - path.delta;
        if (n.g.alpha > lower_bound){
            //std::cerr << "Lower bound: " << n << "\n";
            return lower_bound;
        } 
    }
    return upper_bound;
}

rePEAT::Results rePEAT::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & mdat, double start_time, double end_time){
    double t_ref = start_time;
    std::vector<const SIPPState<Location> *> path;
    // CompoundATF<std::vector<const SIPPState<Location> *>> solutions;
    rePEAT::Results res;
    auto& solutions = res.any_start_time_plan;
    struct timespec ts1, ts2;
    while(t_ref < end_time){
        //std::cerr << "tref: " << t_ref << "\n";
        res.reference_times.push_back(t_ref);
        auto&  m = res.search_metadata.emplace_back();
        asipp::Open<rePEAT::AtSIPPNodeComp> open_list;
        EdgeATF init = EdgeATF(t_ref, t_ref, std::numeric_limits<double>::infinity(), 0.0);
        open_list.emplace(init, eightWayDistance(dest, source->configuration), source, nullptr, nullptr);
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        auto res = asipp::search_core(g, open_list, dest, m);
        if(res.second.beta > 0){
            solutions.insert(res.second, res.first);
            t_ref = update_reference_time(res.second, open_list);
        }
        else{
            t_ref = end(source->safe_interval);
        }
        
        //std::cerr << "tref: " << t_ref << " " << end(source->safe_interval) <<"\n";
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        m.search_time = 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
    }
    return res;
}