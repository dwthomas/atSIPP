#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <functional>
#include "data_structures/structs.hpp"
#include "search_algorithms/sippgraph.hpp"


namespace sipp{
    struct Node;

    struct Node{
        double g;
        double f;
        long state;
        Node() = default;
        Node(double _g, double _h, const long _node):g(_g),f(_g + _h),state(_node){}

        inline friend bool operator>(const Node& a, const Node& b){
            if(a.f == b.f){
                return a.g < b.g;
            }
            return a.f > b.f;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const Node& n){
            stream << n.state << " g:" << n.g << ", f:" << n.f;
            return stream;
        }
    };

    struct NodeComp{
        bool operator()(const Node * a, const Node * b){
            return *a > *b;
        }
    };

    using Queue = boost::heap::d_ary_heap<Node, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<Node>>>;
    typedef typename Queue::handle_type handle_t;
    struct Open{
        Queue queue;
        std::unordered_map<long, long> parent;
        std::unordered_map<long, handle_t> handles;
        std::unordered_map<long, double> expanded;

        inline void emplace(double g, double h, const long n, const long p){
            parent[n] = p;
            handles[n] = queue.push(Node(g, h, n));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline Node top() const{
            return queue.top();
        }

        inline void pop(){
            Node n = top();
            expanded[n.state] = n.g;
            queue.pop();
        }

        inline void decrease_key(handle_t handle ,double g, double h, long n, long p){
            parent[n] = p;
            queue.update(handle, Node(g, h, n));
        }
    };


   std::vector<const SIPPState<Location> *> search(const SippGraph<Location>& g, long source, const Location& dest, MetaData & m, double start_time = 0.0);
}

