#pragma once

#include "data_structures/structs.hpp"
#include "map.hpp"
#include "search_algorithms/sippgraph.hpp"

SippGraph<Location> make_random_sipp_graph(const Map& map, double until,  double occupancy, double min_duration, double max_duration, std::size_t seed=0);

SIPPState<Location>* mark_safe(SippGraph<Location>& g, const Location& loc);