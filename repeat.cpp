#include "repeat.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

double update_reference_time(const EdgeATF& path, rePEAT::Open& open_list){
    double upper_bound = path.beta;
    double lower_bound = path.alpha;
    while(lower_bound < upper_bound){
        if(open_list.empty()){
            return std::numeric_limits<double>::infinity();
        }
        auto n = open_list.top();
        open_list.pop();
        lower_bound = n.f - path.delta;
        if (n.g.alpha > lower_bound){
            return lower_bound;
        } 
    }
    return upper_bound;
}

void rePEAT::search(GraphNode * source, const Location& dest, MetaData & m, double start_time){
    double t_ref = start_time;
    CompoundATF solutions;
    m.init();

    while(true){
        std::cerr << "tref: " << t_ref << "\n";
        Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t_ref, std::numeric_limits<double>::infinity(), 0.0), eightWayDistance(dest, source->state.loc), source, nullptr);
        auto res = asipp::search_core(open_list, dest, m);
        t_ref = update_reference_time(res.second, open_list);
    }
}