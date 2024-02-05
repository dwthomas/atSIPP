#pragma once
#include "atf.hpp"
#include "sippgraph.hpp"
#include "structs.hpp"
#include <boost/multiprecision/detail/number_base.hpp>

template <typename Configuration_t>
struct AtSIPPEdge{
    SIPPState<Configuration_t> * source;
    SIPPState<Configuration_t> * destination;
    EdgeATF duration;


    AtSIPPEdge() = default;
    AtSIPPEdge(SIPPState<Configuration_t> * src, SIPPState<Configuration_t>* dst, EdgeATF dur):source(src),destination(dst),duration(dur){}

    constexpr bool operator ==(const AtSIPPEdge& s) const{
        return *s.source == *source && *s.destination == *destination;
    }

    friend std::size_t hash_value(const AtSIPPEdge& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, *s.source);
        boost::hash_combine(seed, *s.destination);
        return seed;
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const AtSIPPEdge& s){
        stream << *s.source << " -> "  << *s.destination << ": " << s.duration;
        return stream;
    }
};

template <typename Configuration_t>
AtSIPPEdge<Configuration_t> compile(const SIPPEdge<Configuration_t>& sipp_edge){
    const SIPPState<Configuration_t>& u = *sipp_edge.source;
    const SIPPState<Configuration_t>& v = *sipp_edge.destination;
    double zeta = u.safe_interval.lower();
    double alpha = std::max(
        sipp_edge.safe_interval.lower(),
        std::max(
            u.safe_interval.lower(),
            v.safe_interval.lower()-sipp_edge.duration
        )
    );
    double beta = std::min(
        sipp_edge.safe_interval.upper(),
        std::min(
            u.safe_interval.upper(),
            v.safe_interval.upper() - sipp_edge.duration
        )
    );
    double delta = sipp_edge.duration;
    EdgeATF e(zeta, alpha, beta, delta);
    return AtSIPPEdge<Configuration_t>(sipp_edge.source, sipp_edge.destination, e);
}

template <typename Configuration_t>
struct AtSippGraph{
    SippGraph<Configuration_t> sipp_graph;
    std::unordered_map<SIPPState<Configuration_t>, std::vector<AtSIPPEdge<Configuration_t>>> successors;
    std::unordered_map<SIPPState<Configuration_t>, std::vector<AtSIPPEdge<Configuration_t>>> predecessors;

    AtSippGraph() = default;

    AtSippGraph(const SippGraph<Configuration_t>& g):sipp_graph(g){
        for(auto s: sipp_graph.vertices){
            const auto& succ = sipp_graph.successors[s];
            successors[s].reserve(succ.size());
            for (const auto& successor: succ){
                successors[s].emplace_back(compile(successor));
            }
            const auto& pred = sipp_graph.predecessors[s];
            predecessors[s].reserve(pred.size());
            for (const auto& predecessor: pred){
                predecessors[s].emplace_back(compile(predecessor));
            }
        }
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const AtSippGraph& g){
        for(auto s: g.sipp_graph.vertices){
            stream << s << "\n";
            for (auto succ: g.successors.at(s)){
                stream << "\t" << succ << "\n";
            }
        }
        return stream;
    }
};