#pragma once
#include <boost/heap/binomial_heap.hpp>
#include <unordered_map>
#include "graph.hpp"

namespace sipp{
    struct Node;
    using Open_t = boost::heap::binomial_heap<Node, std::less<Node>>;
    using Closed_t = std::unordered_map<Location, double>;

    struct Node{
        double g;
        double f;
        GraphNode* node;
        GraphEdge* parent;
        Node() = default;
        Node(double _g, double _h, const GraphNode * _node, const GraphEdge * _parent):g(_g),f(_g + _h),node(_node),parent(_parent){}

        inline friend bool operator<(const Node& a, const Node& b){
            return a.f < b.f;
        }
    };

    Node search(const Graph& g, const GraphNode& source, const GraphNode& dest);
}

