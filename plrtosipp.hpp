#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include "graph.hpp"



namespace plrtosipp{
    struct Node;

    struct Node{
        EdgeATF g;
        double f;
        GraphNode * node;
        Node() = default;
        Node(EdgeATF e, double _h, GraphNode * _node):g(e),f(e.earliest_arrival_time() + _h),node(_node){}

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
        GraphNode * s;
        double h;
        DijkstraNode() = default;
        DijkstraNode(GraphNode * st, double _h):s(st),h(_h){}

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



    extern std::unordered_map<Location, double> h_static;
    extern std::unordered_map<const GraphNode *, CompoundATF<GraphNode *>> h_dynamic;

    inline double get_h_s(const GraphNode& cur, const Location& dest){
        if(!plrtosipp::h_static.contains(cur.state.loc)){
            plrtosipp::h_static[cur.state.loc] = eightWayDistance(cur.state.loc, dest);
        }
        return plrtosipp::h_static[cur.state.loc];
    }

    inline double get_h(const GraphNode& cur, double cur_t, const Location& dest){
        double h_s = get_h_s(cur, dest);
        if (!plrtosipp::h_dynamic.contains(&cur)){
            return h_s;
        }   
        const auto & catf = plrtosipp::h_dynamic[&cur];
        return catf.arrival_time(cur_t) + get_h_s(*catf.payload_at(cur_t), dest);
    }

    inline void set_h_s(const Location& loc, double x){
        plrtosipp::h_static[loc] = x;
    }

    inline void add_h_dyn(const GraphNode& cur, EdgeATF patf, GraphNode * frontiern){
        if(!plrtosipp::h_dynamic.contains(&cur)){
            plrtosipp::h_dynamic[&cur] = CompoundATF<GraphNode *>(nullptr);
        }
        //std::cerr << "patf" << patf << "\n";
        //std::cerr << "catf" << h_dynamic[&cur] << "\n";
        plrtosipp::h_dynamic[&cur].add(patf, frontiern);
    }

    inline bool isGoal(const GraphNode& cur, const Location& dest){
        return cur.state.loc == dest;
    }

    struct Open{
        Queue queue;
        std::unordered_map<GraphNode *, GraphNode *> parent;
        std::unordered_map<GraphNode *, handle_t> handles;
        std::unordered_map<GraphNode *, double> expanded;

        inline void emplace(EdgeATF e, double h, GraphNode * n, GraphNode * p){
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

        inline void decrease_key(handle_t handle , EdgeATF e, double h, GraphNode * n, GraphNode * p){
            parent[n] = p;
            queue.increase(handle, Node(e, h, n));
        }
    };
   void dump_h();
   std::vector<GraphNode *> search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time = 0.0);
}



