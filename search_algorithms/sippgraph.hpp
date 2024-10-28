#pragma once
#include <bits/types/time_t.h>
#include <functional>
#include <boost/functional/hash.hpp>
#include <unordered_map>

#include "data_structures/constants.hpp"

template <typename Configuration_t>
struct SIPPState{
    Configuration_t configuration;
    atf::interval_t safe_interval;

    SIPPState() = default;
    SIPPState(const Configuration_t& c, const atf::interval_t& si):configuration(c),safe_interval(si){}

    constexpr bool operator ==(const SIPPState& s) const{
        return s.configuration == configuration && s.safe_interval == safe_interval;
    }

    friend std::size_t hash_value(const SIPPState& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, s.configuration.x());
        boost::hash_combine(seed, s.configuration.y());
        boost::hash_combine(seed, s.safe_interval.lower());
        boost::hash_combine(seed, s.safe_interval.upper());
        return seed;
    }
    
    inline friend std::ostream& operator<< (std::ostream& stream, const SIPPState& s){
        stream << s.configuration << " " << s.safe_interval.lower() << " " << s.safe_interval.upper();
        return stream;
    }
};

namespace std {
    template<typename Configuration_t>
    struct hash<SIPPState<Configuration_t>> {
        inline std::size_t operator()(const SIPPState<Configuration_t>& s) const {
           return hash_value(s);
        }
    };
}

template <typename Configuration_t>
struct SIPPEdge{
    long source;
    long destination;
    atf::time_t duration;
    atf::interval_t safe_interval;


    SIPPEdge() = default;
    SIPPEdge(long src, long dst, atf::time_t dur, atf::interval_t interval):source(src),destination(dst),duration(dur),safe_interval(interval){}

    constexpr bool operator ==(const SIPPEdge& s) const{
        return s.source == source && s.destination == destination && safe_interval == s.safe_interval;
    }

    friend std::size_t hash_value(const SIPPEdge& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, s.source);
        boost::hash_combine(seed, s.destination);
        boost::hash_combine(seed, s.safe_interval.lower());
        boost::hash_combine(seed, s.safe_interval.upper());
        return seed;
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const SIPPEdge& s){
        stream << s.source << " -> "  << s.destination << " [" << s.safe_interval.lower() << "," << s.safe_interval.upper() << "): " << s.duration;
        return stream;
    }
};

template <typename Configuration_t>
struct SippGraph{
    std::vector<SIPPState<Configuration_t>> vertices;
    std::unordered_map<long, std::vector<SIPPEdge<Configuration_t>>> successors;
    std::unordered_map<long, std::vector<SIPPEdge<Configuration_t>>> predecessors;
    std::unordered_multimap<Configuration_t, long, std::hash<Configuration_t>> vertex_key;

    SippGraph() = default;
    
    SippGraph(SippGraph& other):vertices(other.vertices){
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const SippGraph& g){
        for(long i =0 ; i < (long)g.vertices.size(); i++){
            stream << g.vertices[i] << "\n";
            for (const auto& succ: g.successors.at(i)){
                stream << "\t" << succ << "\n";
            }
        }
        return stream;
    }
};