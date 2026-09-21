#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <functional>
#include <unordered_map>
#include "data_structures/atf.hpp"
#include "data_structures/constants.hpp"
#include "search_algorithms/newatsippgraph.hpp"
#include "search_algorithms/sippgraph.hpp"

namespace absipp{
    template <class NodeComp>
    struct Node;

    template <class NodeComp>
    using Queue = boost::heap::d_ary_heap<Node<NodeComp>, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<NodeComp>>;
        
    double constexpr h_eight_way_helper(const SIPPState<Location>& cur, double cur_t, const Location& dest){
        (void) cur_t;
        return eightWayDistance(cur.configuration, dest);
    }

    template <class NodeComp>
    struct Node{
        EdgeATF g;
        double f;
        const SIPPState<Location> * state;
        const AtSIPPEdge<Location> * tla;
        const SIPPState<Location> * parent;
        Queue<NodeComp>::handle_type handle;
        Node() = default;
        Node(EdgeATF e, double _h, const SIPPState<Location> * _state, const AtSIPPEdge<Location> * _tla):g(e),f(e.earliest_arrival_time() + _h),state(_state),tla(_tla){}

        Node(double _f, EdgeATF e, const SIPPState<Location> * _state, const AtSIPPEdge<Location> * _tla):g(e),f(_f),state(_state),tla(_tla){}

        // inline friend bool operator>(const Node& a, const Node& b){
        //     if(a.f == b.f){
        //         return a.g < b.g;
        //     }
        //     return a.f > b.f;
        // }

        inline friend std::ostream& operator<< (std::ostream& stream, const Node& n){
            stream << *n.state << " g:" << n.g << ", f:" << n.f << " hex:" << std::hex << n.f << std::dec;
            return stream;
        }
    };

    struct ABNodeComp{
        inline bool operator()(const Node<ABNodeComp>& a, const Node<ABNodeComp>& b) const{
            if(a.f == b.f){
                if(a.g.alpha == b.g.alpha){
                    return a.g.beta < b.g.beta;
                }
                return a.g.alpha < b.g.alpha;
            }
            return a.f > b.f;
        }
    };

    // struct Ghost{
    //     EdgeATF e; 
    //     double h; 
    //     SIPPState<Location> * n; 
    //     SIPPState<Location> * p;
    //     Ghost(EdgeATF _e, double _h, SIPPState<Location> * _n, SIPPState<Location> * _p):e(_e),h(_h),n(_n),p(_p){}
    // };

    template <class NodeComp = ABNodeComp>
    struct Open{
        

        Queue<NodeComp> queue;
        // std::unordered_map<const SIPPState<Location> *, const SIPPState<Location> *> parent;
        // std::unordered_map<const SIPPState<Location> *, handle_t> handles;
        std::unordered_map<const SIPPState<Location> *, const Node<NodeComp> *> expanded;
        std::unordered_multimap<const SIPPState<Location> *, const Node<NodeComp> *> generated; // fix todo!!

        Open(){
            queue.reserve(n_prealloc());
            // parent.reserve(n_prealloc());
            // handles.reserve(n_prealloc());
            expanded.reserve(n_prealloc());
            generated.reserve(n_prealloc());
        }

        inline void emplace(EdgeATF e, double h, const SIPPState<Location> * n, const SIPPState<Location> * p, const AtSIPPEdge<Location> * tla){
            auto nod = queue.push(Node<NodeComp>(e, h, n, tla));
            (*nod).parent = p;
            (*nod).handle = nod;
            generated.emplace(n, &(*nod));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline Node<NodeComp> top() const{
            return queue.top();
        }

        inline void pop(){
            expanded[top().state] = &queue.top();
            queue.pop();
        }

        inline void decrease_key(Queue<NodeComp>::handle_type handle , EdgeATF e, double h, const SIPPState<Location> * n, const SIPPState<Location> * p, const AtSIPPEdge<Location> * tla){
            handle->parent = p;
            queue.update(handle, Node<NodeComp>(e, h, n, tla));
        }
    };

    template <typename Node_t = Node<ABNodeComp>>
    bool isGoal(const Node_t& n, const Location& goal_loc){
        return n.state->configuration == goal_loc;
    }


    template <typename Node_t = Node<ABNodeComp>, typename Open_t = Open<ABNodeComp>> // todo: fix
    std::pair<std::vector<const SIPPState<Location> *>, double> backup(const Node_t& n, Open_t& open_list){
        std::vector<const SIPPState<Location> *> res;
        const SIPPState<Location>* cur = n.state;
        res.push_back(cur);
        cur = n.parent;
        // dump_open(open_list);
        return std::make_pair(res, n.f);
        while(cur != nullptr){
            std::cerr << "backtrack " << *cur << "\n";
            res.push_back(cur);
            cur = open_list.expanded.at(cur)->parent;
        }
        std::reverse(res.begin(), res.end());
        //std::cout << "Arrival time: " << n.f << " " << n << "\n";
        return std::make_pair(res, n.f);
    }

    template <typename Node_t = Node<ABNodeComp>, typename Open_t = Open<ABNodeComp>>
    inline void expand(const AtSippGraph<Location>& g, const Node_t& cur, Open_t& open_list, const Location& goal_loc, MetaData & m, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        m.expanded++;
        double zeta = cur.g.zeta;
        open_list.expanded[cur.state] = &cur;
        // std::cerr << cur.state << *cur.state << "\n";
        for(const AtSIPPEdge<Location>& successor: g.successors.at(cur.state)){
            if(cur.g.earliest_arrival_time() >= successor.duration.beta || cur.g.supremum_arrival_time() <= successor.duration.zeta){
                continue;
            } 
            auto tla = cur.tla;
            if (tla == nullptr){
                tla = &successor;
            }
            double alpha = std::max(cur.g.alpha, successor.duration.alpha - cur.g.delta);
            double beta = std::min(cur.g.beta, successor.duration.beta - cur.g.delta);
            double delta = successor.duration.delta + cur.g.delta;
            EdgeATF arrival_time_function(zeta, alpha, beta, delta);
            if(open_list.expanded.contains(successor.destination)){
                continue;
            }
            else if (open_list.generated.contains(successor.destination)){
                bool is_dominated = false;
                auto range = open_list.generated.equal_range(successor.destination);
                for (auto it = range.first; it != range.second; ++it){
                    auto handle = it->second;
                    ComparisonResult dominates = weak_dominance(arrival_time_function, ((*handle)).g);
                    if (dominates == IS_DOMINATED){
                        is_dominated = true;
                        break;
                    }
                }
                if (is_dominated){
                    continue;
                }
                for (auto it = range.first; it != range.second; ++it){  // Still todo???  todo fix generated as you remove stuff  or just do it lazily!
                    auto handle = it->second;
                    ComparisonResult dominates = weak_dominance(arrival_time_function, ((*handle)).g);
                    if (dominates == DOMINATES){
                        open_list.queue.erase((*it).second->handle); // issue here
                        // open_list.generated.erase((*handle).state);
                    }
                }
            }
            m.generated++;
            double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
            //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
            open_list.emplace(arrival_time_function, h, successor.destination, successor.source, tla);
                
            }
        }
    

    // template <typename Node_t, typename Open_t>
    // inline void expand_noprune(const Node_t& cur, Open_t& open_list, std::vector<Ghost>& ghost_open, const Location& goal_loc, MetaData & m, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
    //     m.expanded++;
    //     double zeta = cur.g.zeta;
    //     for(GraphEdge * successor: cur.node->successors){
    //         if(cur.g.earliest_arrival_time() >= successor->edge.beta || cur.g.supremum_arrival_time() <= successor->edge.zeta){
    //             continue;
    //         } 
    //         double alpha = std::max(cur.g.alpha, successor->edge.alpha - cur.g.delta);
    //         double beta = std::min(cur.g.beta, successor->edge.beta - cur.g.delta);
    //         double delta = successor->edge.delta + cur.g.delta;
    //         EdgeATF arrival_time_function(zeta, alpha, beta, delta);
    //         if(open_list.expanded.contains(successor->destination)){
    //             ghost_open.emplace_back(cur.g, cur.f-cur.g.earliest_arrival_time(), cur.node, open_list.parent[cur.node]);
    //             continue;
    //         }
    //         else if (open_list.handles.contains(successor->destination)){
    //             auto handle = open_list.handles[successor->destination];
    //             if(arrival_time_function.earliest_arrival_time() < (*handle).g.earliest_arrival_time()){
    //                 m.decreased++;
    //                 double h = hf(*successor->destination, arrival_time_function.earliest_arrival_time(), goal_loc);
    //                 //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
    //                 open_list.decrease_key(handle ,arrival_time_function, h, successor->destination, successor->source);
    //             }
    //         }
    //         else{
    //             m.generated++;
    //             double h = hf(*successor->destination, arrival_time_function.earliest_arrival_time(), goal_loc);
    //             //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
    //             open_list.emplace(arrival_time_function, h, successor->destination, successor->source);
    //             //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
    //         }
    //     }
    // }

    template<typename Open_t = Open<ABNodeComp>>
    inline void dump_open(const Open_t& open_list){
        auto cur = open_list.queue.ordered_begin();
        auto end = open_list.queue.ordered_end();
        std::cerr << "Open:\n";
        while(cur != end){
            std::cerr << "\t" << *cur << "\n";
            cur = std::next(cur);
        }
        std::cerr << "Generated:\n";
        for( auto x : open_list.generated){
            std::cerr <<  "\t" << *x.first << "\n";
        }

        std::cerr << "Expanded:\n";
        for( auto x : open_list.expanded){
            std::cerr <<  "\t" << *x.first << " " << *x.second->state << "\n";
        }
    }
    template<typename Open_t = Open<ABNodeComp>>
    inline std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> search_core(const AtSippGraph<Location>& g, Open_t& open_list, const Location& dest, MetaData & m, long expansion_budget = -1, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        long start_expansions = m.expanded;
        //m.search_timer.start();
        while(!open_list.empty()){
           //dump_open(open_list);
            auto cur = open_list.top();
            //std::cout << *cur.node << "\n";
           // std::cerr << expansion_budget << "\n";
            //std::cerr << isGoal(cur, dest) << " " << (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget) << "\n";
            if(isGoal(cur, dest) || (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget)){
                //m.search_timer.stop();

                return std::make_pair(backup(cur, open_list).first, cur.g);
            }
            open_list.pop();
            expand(g, cur, open_list, dest, m, hf);
        }
        //std::cerr << "Failed to find path\n";
        return std::make_pair(std::vector<const SIPPState<Location> *>(), EdgeATF());
        // std::cerr << "Failed to find path\n";
        // exit(-1);
    }

    // template<typename Open_t>
    // inline void search_core_noprune(Open_t& open_list, const Location& dest, MetaData & m, long expansion_budget = -1, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
    //     long start_expansions = m.expanded;
    //     std::vector<Ghost> ghost_open;
    //     while(!open_list.empty()){
    //         //dump_open(open_list);
    //         auto cur = open_list.top();
    //         //std::cout << *cur.node << "\n";
    //         if(isGoal(cur, dest) || (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget)){
    //             return;
    //         }
    //         open_list.pop();
    //         expand_noprune(cur, open_list, ghost_open, dest, m, hf);
    //     }
    //     for (auto ghost: ghost_open){
    //         open_list.emplace(ghost.e, ghost.h, ghost.n, ghost.p);
    //     }
    // }

    std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time = 0.0, long expansion_budget = -1, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper);
}

