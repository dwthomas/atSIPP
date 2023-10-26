#pragma once
#include <boost/heap/binomial_heap.hpp>
#include "graph.hpp"


struct Node;
using Open_t = boost::heap::binomial_heap<Node, std::less>;

struct Node{
    double g;
    double f;
    GraphNode* node;
    GraphEdge* parent;
    State() = default;
    State(double _g, double _h, GraphNode * _node, GraphEdge * _parent):g(_g),f(_g + _h),node(_node),parent(_parent){}

    inline friend bool operator<(const State& a, const State& b){
        return a.f < b.f;
    }
}

