#include "augmentedsipp.hpp"
#include "atsippgraph.hpp"
#include "structs.hpp"
#include <algorithm>
#include <limits>
#include <utility>

using namespace asipp;

std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> asipp::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time, long expansion_budget, double (*hf)(const SIPPState<Location>&, double , const Location& )){
    Open open_list;
    m.init();
    open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), start_time, std::numeric_limits<double>::infinity(), 0.0), eightWayDistance(dest, source->configuration), source, nullptr);
    return search_core(g, open_list, dest, m, expansion_budget, hf);
}