#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include "atsippgraph.hpp"
#include "augmentedsipp.hpp"
#include "sippgraph.hpp"

namespace rtasipp{
    // struct Node;

    // struct Node{
    //     EdgeATF g;
    //     double f;
    //     AtsippGraphNode * node;
    //     GraphEdge * tla;
    //     Node() = default;
    //     Node(const EdgeATF& e, double _h, AtsippGraphNode * _node, GraphEdge * _tla):g(e),f(e.earliest_arrival_time() + _h),node(_node),tla(_tla){}

    //     inline friend bool operator>(const Node& a, const Node& b){
    //         if(a.f == b.f){
    //             if(a.g.alpha == b.g.alpha){
    //                 return a.g.beta < b.g.beta;
    //             }
    //             return a.g.alpha < b.g.alpha;
    //         }
    //         return a.f > b.f;
    //     }

    //     inline friend std::ostream& operator<< (std::ostream& stream, const Node& n){
    //         stream << *n.node << " g:" << n.g << ", f:" << n.f;
    //         return stream;
    //     }
    // };

    // struct NodeComp{
    //     bool operator()(const Node * a, const Node * b){
    //         return *a > *b;
    //     }
    // };
    // using Queue = boost::heap::d_ary_heap<Node, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<Node>>>;
    // typedef typename Queue::handle_type handle_t;
    using CATF = CompoundATF<std::nullptr_t>;
    extern std::unordered_map<Location, double> h_static;
    extern std::unordered_map<const SIPPState<Location> *, CATF> h_dynamic;

    inline double get_h_s(const SIPPState<Location>* cur, const Location& dest){
        if(h_static.find(cur->configuration) == h_static.end()){
            h_static[cur->configuration] = eightWayDistance(cur->configuration, dest);
        }
        return h_static[cur->configuration];
    }

    inline double get_h(const SIPPState<Location>& cur, double cur_t, const Location& dest){
        double h_s = get_h_s(&cur, dest);
        if (h_dynamic.find(&cur) == h_dynamic.end()){
            return h_s;
        }   
        const auto & catf = h_dynamic[&cur];
        //std::cerr << catf;
        //return catf.arrival_time(cur_t) + get_h_s(catf.payload_at(cur_t), dest);
        return catf.arrival_time(cur_t);
    }

    inline void set_h_s(const Location& loc, double x){
        h_static[loc] = x;
    }

    inline void add_h_dyn(const SIPPState<Location> * cur, EdgeATF patf){
        if(h_dynamic.find(cur) == h_dynamic.end()){
            h_dynamic[cur] = CATF();
        }
        h_dynamic[cur].insert(patf, nullptr);
    }

    // inline bool isGoal(const AtsippGraphNode& cur, const Location& dest){
    //     return cur.state.loc == dest;
    // }

    // struct Open{
    //     Queue queue;
    //     std::unordered_map<AtsippGraphNode *, AtsippGraphNode *> parent;
    //     std::unordered_map<AtsippGraphNode *, handle_t> handles;
    //     std::unordered_map<AtsippGraphNode *, double> expanded;

    //     inline void emplace(EdgeATF e, double h, AtsippGraphNode * n, AtsippGraphNode * p, GraphEdge * tla){
    //         parent[n] = p;
    //         handles[n] = queue.push(Node(e, h, n, tla));
    //     }

    //     inline bool empty() const{
    //         return queue.empty();
    //     }

    //     inline Node top() const{
    //         return queue.top();
    //     }

    //     inline void pop(){
    //         Node n = top();
    //         expanded[n.node] = n.g.earliest_arrival_time();
    //         queue.pop();
    //     }

    //     inline void decrease_key(handle_t handle , EdgeATF e, double h, AtsippGraphNode * n, AtsippGraphNode * p, GraphEdge * tla){
    //         parent[n] = p;
    //         queue.increase(handle, Node(e, h, n, tla));
    //     }
    // };
    inline void dump_h_s(std::unordered_map<Location, double> h_static){
        std::cerr << "h_s\n";
        for (auto x: h_static){
            std::cerr << x.first << ": " << x.second << "\n";
        }
        std::cerr << "\n";
    }

   std::vector<const SIPPState<Location> *> search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time = 0.0, long expansion_budget = -1);
}

