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
        GraphEdge * tla;
        Node() = default;
        Node(const EdgeATF& e, double _h, GraphNode * _node, GraphEdge * _tla):g(e),f(e.earliest_arrival_time() + _h),node(_node),tla(_tla){}


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
        EdgeATF g;
        GraphNode * node;
        DijkstraNode() = default;
        DijkstraNode(EdgeATF e, GraphNode * _node):g(e),node(_node){}

        inline friend bool operator>(const DijkstraNode& a, const DijkstraNode& b){
            return a.g.earliest_arrival_time() > b.g.earliest_arrival_time();
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const DijkstraNode& n){
            stream << *n.node << " g:" << n.g;
            return stream;
        }
    };

    struct DijkstraNodeComp{
        bool operator()(const DijkstraNode * a, const DijkstraNode * b){
            return *a > *b;
        }
    };
    using DijkstraQueue = boost::heap::d_ary_heap<DijkstraNode, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<DijkstraNode>>>;
    typedef typename DijkstraQueue::handle_type Dijkstra_handle_t;


    using CATF = CompoundATF<std::nullptr_t>;
    extern std::unordered_map<Location, double> h_static;
    extern std::unordered_map<const GraphNode *, CATF> h_dynamic;

    inline void dump_h_s(std::unordered_map<Location, double> h_static){
        std::cerr << "h_s\n";
        for (auto x: h_static){
            std::cerr << x.first << ": " << x.second << "\n";
        }
        std::cerr << "\n";
    }

    inline double get_h_s(const Location& cur_loc, const Location& dest){
        if(h_static.find(cur_loc) == h_static.end()){
            h_static[cur_loc] = eightWayDistance(cur_loc, dest);
        }
        return h_static[cur_loc];
    }

    inline double get_h_s(const GraphNode& cur, const Location& dest){
        return get_h_s(cur.state.loc, dest);
    }

    inline double get_h(const GraphNode& cur, double cur_t, const Location& dest){
        double h_s = get_h_s(cur, dest);
        if (h_dynamic.find(&cur) == h_dynamic.end()){
            return h_s;
        }   
        const auto & catf = h_dynamic[&cur];
        //return catf.arrival_time(cur_t) + get_h_s(catf.payload_at(cur_t), dest);
        return catf.arrival_time(cur_t);
    }

    inline void set_h_s(const Location& loc, double x){
        h_static[loc] = x;
    }

    inline void add_h_dyn(const GraphNode& cur, EdgeATF patf){
        if(h_dynamic.find(&cur) == h_dynamic.end()){
            h_dynamic[&cur] = CATF();
        }
        //std::cerr << "patf" << patf << "\n";
        //std::cerr << "catf" << h_dynamic[&cur] << "\n";
        h_dynamic[&cur].insert(patf, nullptr);
    }

    inline bool isGoal(const GraphNode& cur, const Location& dest){
        return cur.state.loc == dest;
    }

    struct Open{
        Queue queue;
        std::unordered_map<GraphNode *, GraphNode *> parent;
        std::unordered_map<GraphNode *, handle_t> handles;
        std::unordered_map<GraphNode *, double> expanded;

        inline void emplace(EdgeATF e, double h, GraphNode * n, GraphNode * p, GraphEdge * tla){
            parent[n] = p;
            handles[n] = queue.push(Node(e, h, n, tla));
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

        inline void decrease_key(handle_t handle , EdgeATF e, double h, GraphNode * n, GraphNode * p, GraphEdge * tla){
            parent[n] = p;
            queue.increase(handle, Node(e, h, n, tla));
        }
    };

    struct DijkstraOpen{
        DijkstraQueue queue;
        //std::unordered_map<GraphNode *, GraphNode *> parent;
        std::unordered_map<GraphNode *, Dijkstra_handle_t> handles;

        inline void emplace(EdgeATF e, GraphNode * n){
            handles[n] = queue.push(DijkstraNode(e,  n));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline DijkstraNode top() const{
            return queue.top();
        }

        inline void pop(){
            //DijkstraNode n = top();
            //expanded[n.node] = n.g.earliest_arrival_time();
            queue.pop();
        }

        inline void decrease_key(Dijkstra_handle_t handle , EdgeATF e, GraphNode * n){
            queue.update(handle, DijkstraNode(e, n));
        }
    };

    struct LSSNode{
        double g;
        GraphNode * node;
        LSSNode() = default;
        LSSNode(double _g,  GraphNode * _node):g(_g),node(_node){}

        inline friend bool operator>(const LSSNode& a, const LSSNode& b){
            return a.g > b.g;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const LSSNode& n){
            stream << *n.node << " g:" << n.g;
            return stream;
        }
    };

    struct LSSNodeComp{
        bool operator()(const LSSNode * a, const LSSNode * b){
            return *a > *b;
        }
    };
    using LSSQueue = boost::heap::d_ary_heap<LSSNode, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<LSSNode>>>;
    typedef typename LSSQueue::handle_type LSS_handle_t;

    struct LSSOpen{
        LSSQueue queue;
        //std::unordered_map<GraphNode *, GraphNode *> parent;
        std::unordered_map<GraphNode *, LSS_handle_t> handles;

        inline void emplace(double g, GraphNode * n){
            handles[n] = queue.push(LSSNode(g, n));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline LSSNode top() const{
            return queue.top();
        }

        inline void pop(){
            //DijkstraNode n = top();
            //expanded[n.node] = n.g.earliest_arrival_time();
            queue.pop();
        }

        inline void decrease_key(LSS_handle_t handle , double g, GraphNode * n){
            queue.increase(handle, LSSNode(g, n));
        }
    };

    void lsslrtsipp(const Open& open_list, const Location& dest, MetaData& m);
    void plrtolearn(const Open& open_list, const Location& dest, MetaData& m);
    std::vector<GraphNode *> search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time = 0.0);
}

