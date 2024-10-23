#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include "newatsippgraph.hpp"
#include "data_structures/structs.hpp"

namespace hybrid{
    std::vector<const SIPPState<Location> *> search(const AtSippGraph<Location> & g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time = 0.0);
}

