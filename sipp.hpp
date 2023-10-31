#pragma once
#include <boost/heap/binomial_heap.hpp>
#include <unordered_map>
#include "graph.hpp"

namespace sipp{
    struct Node;

    struct Node{
        double g;
        double f;
        GraphNode* node;
        GraphEdge* parent;
        Node() = default;
        Node(double _g, double _h, GraphNode * _node, GraphEdge * _parent):g(_g),f(_g + _h),node(_node),parent(_parent){}

        inline friend bool operator>(const Node& a, const Node& b){
            return a.f > b.f;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const Node& n){
            stream << *n.node << " g:" << n.g << ", f:" << n.f;
            return stream;
        }
    };
    using Open_t = boost::heap::binomial_heap<Node, boost::heap::compare<std::greater<Node> >>;


    Node search(GraphNode * source, const Location& dest);
}

