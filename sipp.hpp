#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include "graph.hpp"

namespace sipp{
    struct Node;

    struct Node{
        double g;
        double f;
        GraphNode * node;
        Node() = default;
        Node(double _g, double _h, GraphNode * _node):g(_g),f(_g + _h),node(_node){}

        inline friend bool operator>(const Node& a, const Node& b){
            return a.f > b.f;
        }

        inline friend bool operator>(const Node * a, const Node * b){
            return a->f > b->f;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const Node& n){
            stream << *n.node << " g:" << n.g << ", f:" << n.f;
            return stream;
        }
    };

    struct Open{
        boost::heap::d_ary_heap<Node *, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<Node *>>> queue;
        std::unordered_map<GraphNode *, GraphNode *> parent;
    };


    Node search(GraphNode * source, const Location& dest);
}

