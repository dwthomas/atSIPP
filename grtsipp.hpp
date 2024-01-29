#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include "atsippgraph.hpp"



namespace grtsipp{
    struct Node;

    struct Node{
        EdgeATF g;
        double f;
        AtsippGraphNode * node;
        Node() = default;
        Node(EdgeATF e, double _h, AtsippGraphNode * _node):g(e),f(e.earliest_arrival_time() + _h),node(_node){}

        inline friend bool operator>(const Node& a, const Node& b){
            if(a.f == b.f){
                if(a.g.alpha == b.g.alpha){
                    return a.g.beta < b.g.beta;
                }
                return a.g.alpha < b.g.alpha;
            }
            return a.f > b.f;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const Node& n){
            stream << *n.node << " g:" << n.g << ", f:" << n.f;
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
    
    struct DijkstraNode{
        AtsippGraphNode * s;
        double h;
        DijkstraNode() = default;
        DijkstraNode(AtsippGraphNode * st, double _h):s(st),h(_h){}

        inline friend bool operator>(const DijkstraNode& a, const DijkstraNode& b){
            return a.h > b.h;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const DijkstraNode& n){
            stream << *n.s << " " << n.h;
            return stream;
        }
    };   
    

    struct DijkstraNodeComp{
        bool operator()(const DijkstraNode * a, const DijkstraNode * b){
            return *a > *b;
        }
    };
    using DijkstraQueue = boost::heap::d_ary_heap<DijkstraNode, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<DijkstraNode>>>;
    typedef typename DijkstraQueue::handle_type dijkstra_handle_t;


    extern std::unordered_map<State, double> h;

    inline double get_h(const AtsippGraphNode& cur, double cur_t, const Location& dest){
        (void) cur_t;
        State s = cur.state;
        if (!h.contains(s)){
            h[s] = eightWayDistance(cur.state.loc, dest);
        }   
        return h[s];
    }

    inline void set_h(const State & s,  double new_h){
        h[s] = new_h;
    }

    inline bool isGoal(const AtsippGraphNode& cur, const Location& dest){
        return cur.state.loc == dest;
    }

    struct Open{
        Queue queue;
        std::unordered_map<AtsippGraphNode *, AtsippGraphNode *> parent;
        std::unordered_map<AtsippGraphNode *, handle_t> handles;
        std::unordered_map<AtsippGraphNode *, double> expanded;

        inline void emplace(EdgeATF e, double h, AtsippGraphNode * n, AtsippGraphNode * p){
            parent[n] = p;
            handles[n] = queue.push(Node(e, h, n));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline Node top() const{
            return queue.top();
        }

        inline void pop(){
            Node n = top();
            expanded[n.node] = n.g.earliest_arrival_time();
            queue.pop();
        }

        inline void decrease_key(handle_t handle , EdgeATF e, double h, AtsippGraphNode * n, AtsippGraphNode * p){
            parent[n] = p;
            queue.increase(handle, Node(e, h, n));
        }
    };

   std::vector<AtsippGraphNode *> search(AtsippGraphNode * source, const Location& dest, MetaData & m, long budget, double start_time = 0.0);
}



