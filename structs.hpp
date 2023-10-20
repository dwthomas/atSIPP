#pragma once

#include <cassert>
#include <iostream>
#include <limits>
#include <boost/container/flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/functional/hash.hpp>
#include "constants.hpp"

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

std::size_t hash_value(Location const& loc){
    boost::hash<uint> hasher;
    return hasher(loc.pack());
}

