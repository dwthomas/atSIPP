#include <boost/container/flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>
#include "arrivalTimeFunctions.hpp"

struct GraphEdge;
struct GraphNode;

struct GraphEdge{
    EdgeATF edge;
    GraphNode * destination;
};

struct GraphNode{
    Location loc;
    boost::container::flat_set<GraphEdge *> sucessors;
};

class Graph{
    std::vector<GraphEdge> edges;
    boost::unordered::unordered_flat_map<Location, GraphNode> nodes;
};