#include "augmentedsipp.hpp"
#include "graph.hpp"
#include "structs.hpp"
#include <algorithm>
#include <limits>
#include <utility>

using namespace asipp;

std::pair<std::vector<GraphNode *>, EdgeATF> asipp::search(GraphNode * source, const Location& dest, MetaData & m, double start_time){
    Open open_list;
    m.init();
    open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), start_time, std::numeric_limits<double>::infinity(), 0.0), 0, source, nullptr);
    return search_core(open_list, dest, m);
}