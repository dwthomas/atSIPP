#pragma once

#include <cassert>
#include <iostream>
#include <limits>
#include <boost/container/flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/functional/hash.hpp>
#include "constants.hpp"


using SafeInterval = std::pair<intervalTime_t, intervalTime_t>;

inline intervalTime_t begin(const SafeInterval& si){
    return si.second;
}

inline intervalTime_t end(const SafeInterval& si){
    return si.first;
}

inline bool contains(const SafeInterval& si, intervalTime_t t){
    return begin(si) <= t && t < end(si);
}

inline bool overlap(const SafeInterval& left, const SafeInterval& right){
    auto earliest = std::min(begin(left), begin(right));
    auto latest = std::max(end(left), end(right));
    return latest - earliest <= end(left) - begin(left) + end(right) - begin(right);
}

struct Location{
    private:
        gIndex_t _x;
        gIndex_t _y;
    public:
        Location() = default;
        Location(int xi, int yi):_x(xi),_y(yi){}
        constexpr int x() const{
            return _x;
        }

        constexpr int y() const{
            return _y;
        }

        constexpr bool operator ==(const Location & l) const{
            return x() == l.x() && y() == l.y();
        }
        constexpr uint pack() const{
            return ((uint)_x << 16) + _y;
        }
        inline friend std::ostream& operator<< (std::ostream& stream, const Location& loc){
            stream << loc.x() << " " << loc.y();
            return stream;
        }
};

constexpr double eightWayDistance(const Location& l1, const Location& l2){
    int dx = std::abs(l1.x() - l2.x()); 
    int dy = std::abs(l1.y() - l2.y());
    long diag = std::min(dx, dy);
    long flat = dy + dx - 2*diag;
    return (double)(flat +  sqrt2()*diag);
}


namespace std {
    template<>
    struct hash<Location> {
        inline size_t operator()(const Location& loc) const {
          boost::hash<uint> hasher;
    return hasher(loc.pack());
        }
    };
}

inline std::pair<Location, Location> canonical_edge(const Location& loc1, const Location& loc2){
    Location a(std::min(loc1.x(), loc2.x()), std::min(loc1.y(), loc2.y()));
    Location b(std::max(loc1.x(), loc2.x()), std::max(loc1.y(), loc2.y()));
    return std::pair<Location, Location>(a, b);
}

struct State{
    Location loc;
    SafeInterval interval;
    State() = default;
    State(int a, int b, double s, double e):loc(a,b),interval(e,s){}; 
    constexpr bool operator ==(const State & s) const{
        return loc == s.loc && interval == s.interval;
    }
    inline friend std::ostream& operator<< (std::ostream& stream, const State& s){
        stream << s.loc << " <" << s.interval.second << "," << s.interval.first << ">";
        return stream;
    }
};

namespace std {
    template<>
    struct hash<State> {
        inline std::size_t operator()(const State& s) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, s.loc.pack());
            boost::hash_combine(seed, s.interval.second);
            boost::hash_combine(seed, s.interval.second);
            return seed;
        }
    };
}

struct MetaData{
    long generated;
    long expanded;
    long decreased;

    inline void init(){
        generated = 0;
        expanded = 0;
        decreased = 0;
    }
    inline friend std::ostream& operator<< (std::ostream& stream, const MetaData& m){
        stream << "Nodes generated: " << m.generated << " Nodes decreased: " << m.decreased << " Nodes expanded: " << m.expanded; 
        return stream;
    }
};