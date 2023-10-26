#include "sipp.hpp"

using namespace sipp;

Node search(const Graph& g, const GraphNode& source, const GraphNode& dest){
    Open_t open_list;
    Closed_t closed_until;
    open_list.emplace(0.0, dest - source, &source, nullptr);
    while(!open_list.empty()){
        const Node& cur = open_list.top();
        open_list.pop();
    }
}