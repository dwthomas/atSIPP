#pragma once
#include <bits/types/time_t.h>
#include <boost/functional/hash.hpp>
#include <set>
#include <unordered_map>

#include "constants.hpp"
#include "map.hpp"

template <typename Configuration_t>
struct SIPPState{
    Configuration_t configuration;
    atf::interval_t safe_interval;

    SIPPState() = default;
    SIPPState(const Configuration_t& c, const atf::interval_t& si):configuration(c),safe_interval(si){}

    constexpr bool operator ==(const SIPPState& s) const{
        return s.configuration == configuration && s.time == time;
    }

    friend std::size_t hash_value(const SIPPState& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, s.configuration);
        boost::hash_combine(seed, s.safe_interval);
        return seed;
    }
    
    inline friend std::ostream& operator<< (std::ostream& stream, const SIPPState& s){
        stream << s.configuration << " " << s.time;
        return stream;
    }
};

template <typename Configuration_t>
struct SIPPEdge{
    SIPPState<Configuration_t> * source;
    SIPPState<Configuration_t> * destination;
    atf::time_t duration;


    SIPPEdge() = default;
    SIPPEdge(const SIPPState<Configuration_t> & src, const SIPPState<Configuration_t> & dst, atf::time_t dur):source(src),destination(dst),duration(dur){}

    constexpr bool operator ==(const SIPPEdge& s) const{
        return *s.source == *source && *s.destination == *destination;
    }

    friend std::size_t hash_value(const SIPPEdge& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, *s.source);
        boost::hash_combine(seed, *s.destination);
        return seed;
    }
    
    inline friend std::ostream& operator<< (std::ostream& stream, const SIPPEdge& s){
        stream << *s.source << " -> "  << *s.destination << ": " << s.duration;
        return stream;
    }
};

template <typename Configuration_t>
struct SippGraph{
    std::vector<SIPPState<Configuration_t>> vertices;
    std::vector<SIPPEdge<Configuration_t>> edges;
    std::unordered_map<SIPPState<Configuration_t>, std::set<SIPPEdge<Configuration_t>>> successors;
    std::unordered_map<SIPPState<Configuration_t>, std::set<SIPPEdge<Configuration_t>>> predecessors;

    SippGraph() = default;

    inline friend std::ostream& operator<< (std::ostream& stream, const SippGraph& g){
        for(auto s: g.vertices){
            stream << s << "\n";
            for (auto succ: g.successors[s]){
                stream << "\t" << succ << "\n";
            }
        }
        return stream;
    }
};