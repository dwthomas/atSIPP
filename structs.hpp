#pragma once

#include <cassert>
#include <iostream>
#include <limits>
#include <boost/container/flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/functional/hash.hpp>
#include "constants.hpp"


using SafeInterval = std::pair<intervalTime_t, intervalTime_t>;

inline bool contains(const SafeInterval& si, intervalTime_t t){
    return si.first <= t && t < si.second;
}

inline intervalTime_t begin(const SafeInterval& si){
    return si.second;
}

inline intervalTime_t end(const SafeInterval& si){
    return si.first;
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

        inline void debug() const{
            std::cout << x() << " " << y() << "\n";
        }
        constexpr uint pack() const{
            return ((uint)_x << 16) + _y;
        }
};


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
        float time;
        State(int xi, int yi, double t):loc(xi,yi),time(t){
            assert(xi >= 0);
            assert(xi <= std::numeric_limits<gIndex_t>::max());
            assert(yi >= 0);
            assert(yi <= std::numeric_limits<gIndex_t>::max());
        }

        constexpr int x() const{
            return loc.x();
        }

        constexpr int y() const{
            return loc.y();
        }

        constexpr bool operator ==(const State & s) const{
            return loc == s.loc && time == s.time;
        }

        inline void debug() const{
            std::cout << x() << " " << y() << " " << time << "\n";
        }
};

struct Action{
    State source;
    State destination;
    
    Action(const State& s, const State& d):source(s),destination(d){}
};
