#pragma once
#include "data_structures/atf.hpp"

#include <boost/heap/d_ary_heap.hpp>
#include <limits>
#include "../aSIPPnew/augmentedsipp.hpp"

inline double update_reference_time(const EdgeATF& path, augmentedsipp::Open<augmentedsipp::ABNodeComp>& open_list){
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