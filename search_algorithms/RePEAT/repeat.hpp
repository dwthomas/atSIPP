#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <limits>
#include "data_structures/structs.hpp"
#include "search_algorithms/newatsippgraph.hpp"
#include "../aSIPPnew/augmentedsipp.hpp"


namespace rePEAT{
    // struct AtSIPPNodeComp{
    //     inline bool operator()(const augmentedsipp::Node& a, const augmentedsipp::Node& b) const{
    //         if(a.f == b.f){
    //             if(a.g.alpha == b.g.alpha){
    //                 return a.g.beta < b.g.beta;
    //             }
    //             return a.g.alpha < b.g.alpha;
    //         }
    //         return a.f > b.f;
    //     }
    // };
    struct Results{
        CompoundATF<std::vector<const SIPPState<Location> *>> any_start_time_plan;
        std::vector<double> reference_times;
        std::vector<MetaData> search_metadata;

        inline friend std::ostream& operator<<(std::ostream& os, const Results& res){
            os << "\"results\": [\n";
            for(std::size_t i = 0; i < res.reference_times.size(); i++){
                os << "{";
                os << "\"reference_time\": " << res.reference_times[i] << ",\n";
                os << res.search_metadata[i];
                if(i < res.reference_times.size() - 1){
                    os << "},\n";
                }
                else{
                    os << "}\n";
                }
            }
            os << "]";
            return os;
        }
    };

    rePEAT::Results search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time = 0.0, double end_time = std::numeric_limits<double>::infinity(), bool test_query_time = false);
}
