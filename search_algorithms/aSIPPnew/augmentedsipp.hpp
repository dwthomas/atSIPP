#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <cstddef>
#include <unordered_map>
#include "data_structures/constants.hpp"
#include "search_algorithms/newatsippgraph.hpp"
#include "search_algorithms/sippgraph.hpp"

namespace augmentedsipp{
    struct Node;

    double constexpr h_eight_way_helper(const SIPPState<Location>& cur, double cur_t, const Location& dest){
        (void) cur_t;
        return eightWayDistance(cur.configuration, dest);
    }

    struct Node{
        EdgeATF g;
        double f;
        const SIPPState<Location> * state;
        const AtSIPPEdge<Location> * tla;
        std::size_t parent_index;
        bool pruned;
        Node() = default;
        Node(EdgeATF e, double _h, const SIPPState<Location> * _state, const AtSIPPEdge<Location> * _tla, std::size_t _parent_index):g(e),f(e.earliest_arrival_time() + _h),state(_state),tla(_tla),parent_index(_parent_index), pruned(false){}

        Node(double _f, EdgeATF e, const SIPPState<Location> * _state, const AtSIPPEdge<Location> * _tla):g(e),f(_f),state(_state),tla(_tla), pruned(false){}

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

    struct StandardNodeComp{ // negating and <= matters for decreased count
        inline bool operator()(const Node& a, const Node& b) const{
            if(a.f == b.f){
                return a.g < b.g;
            }
            return a.f > b.f;
        }
    };

    struct NullNodeComp{
        inline bool operator()(const Node& a, const Node& b) const{
            return true;
        }
    };

    // struct Ghost{
    //     EdgeATF e; 
    //     double h; 
    //     SIPPState<Location> * n; 
    //     SIPPState<Location> * p;
    //     Ghost(EdgeATF _e, double _h, SIPPState<Location> * _n, SIPPState<Location> * _p):e(_e),h(_h),n(_n),p(_p){}
    // };

    template <class NodeComp = StandardNodeComp>
    struct Open{

    inline static std::vector<Node> nodes;
        
        struct NodeCompWrapper{
            inline bool operator()(std::size_t a, std::size_t b) const{
                return NodeComp()(Open::nodes[a], Open::nodes[b]);
            }
        };

        using Queue = boost::heap::d_ary_heap<std::size_t, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<NodeCompWrapper>>;
        typedef typename Queue::handle_type handle_t;



        Queue queue;
        std::unordered_map<const SIPPState<Location> *, handle_t> handles;
        std::unordered_multimap<const SIPPState<Location> *, std::size_t> generated;

        Open(){
            nodes.clear();
            nodes.reserve(n_prealloc());
            queue.reserve(n_prealloc());
            handles.reserve(n_prealloc());
            generated.reserve(n_prealloc());
        }

        
        inline void emplace(EdgeATF e, double h, const SIPPState<Location> * n, std::size_t p_index, const AtSIPPEdge<Location> * tla){
            nodes.emplace_back(e, h, n, tla, p_index);
            generated.emplace(n, nodes.size()-1);
            handles[n] = queue.push(nodes.size()-1);
        }

        inline void emplace(EdgeATF e, double h, const SIPPState<Location> * n, const Node& p, const AtSIPPEdge<Location> * tla){
            emplace(e,h,n, &p - &nodes[0], tla);
        }


        inline void fix_top(){
            while(!queue.empty()){
                Node& n = nodes[queue.top()];
                if (n.pruned){
                    queue.pop();
                }
                else{
                    break;
                }
            }
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline Node top() const{
            return nodes[queue.top()];
        }

        inline void pop(){
            Node n = top();
            // expanded[n.state] = n.g.earliest_arrival_time();
            queue.pop();
        }

        inline void decrease_key(handle_t handle , EdgeATF e, double h, const SIPPState<Location> * n,  const Node& p, const AtSIPPEdge<Location> * tla){
            nodes.emplace_back(e, h, n, tla, &p - &nodes[0]);
            queue.update(handle, nodes.size()-1);
        }
    };

    template <typename Node_t>
    bool isGoal(const Node_t& n, const Location& goal_loc){
        return n.state->configuration == goal_loc;
    }


    template <typename Node_t, typename Open_t>
    std::pair<std::vector<const SIPPState<Location> *>, double> backup(const Node_t& n, Open_t& open_list){
        std::cerr << "backing up\n";
        std::vector<const SIPPState<Location> *> res;
        std::size_t cur = &n - &open_list.nodes[0];
        while(cur != open_list.nodes[cur].parent_index){
            // std::cerr << "cur: " << cur << " " << *open_list.nodes[cur].state << "\n";
            res.push_back(open_list.nodes[cur].state);
            cur = open_list.nodes[cur].parent_index;
        }
        res.push_back(open_list.nodes[cur].state);
        std::reverse(res.begin(), res.end());
        //std::cout << "Arrival time: " << n.f << " " << n << "\n";
        return std::make_pair(res, n.f);
    }

    template <typename Open_t, typename Prune_t>
    inline void expand(const AtSippGraph<Location>& g, const Node& cur, Open_t& open_list, const Location& goal_loc, MetaData & m, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        m.expanded++;
        double zeta = cur.g.zeta;
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
            double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
            // if(open_list.expanded.contains(successor.destination)){ // already expanded
            //     continue;
            // }
            // else
            bool generate = true;
            if (open_list.generated.contains(successor.destination)){
                Node n(arrival_time_function, h, successor.destination, tla, &cur - &open_list.nodes[0]);
                auto range = open_list.generated.equal_range(successor.destination);
                for ( auto succ = range.first; succ != range.second; ++succ){
                    Node& n_pre = open_list.nodes[succ->second];
                    if(!Prune_t()(n, n_pre)){// 
                      n_pre.pruned = true;
                    }
                    else if (!Prune_t()(n_pre, n)){
                        generate = false;
                        break;                        
                    }
                }
                // if(arrival_time_function.earliest_arrival_time() < open_list.nodes[(*handle)].g.earliest_arrival_time()){  // prune successor?
                // if(!Prune_t()(n, open_list.nodes[(*handle)])){  // prune successor?
                //     m.decreased++;
                //     //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                //     open_list.decrease_key(handle, arrival_time_function, h, successor.destination, cur, tla);
                // }
                // otherwise better than current?
            }

            if (generate){
                m.generated++;
                //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                open_list.emplace(arrival_time_function, h, successor.destination, cur, tla);
                //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
            }
        }
        open_list.fix_top();
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

    template<typename Open_t>
    inline void dump_open(const Open_t& open_list){
        auto cur = open_list.queue.ordered_begin();
        auto end = open_list.queue.ordered_end();
        std::cerr << "Open:\n";
        while(cur != end){
            std::cerr << "\t" << *cur << "\n";
            cur = std::next(cur);
        }
        std::cerr << "Closed:\n";
        for( auto x : open_list.generated){
            std::cerr <<  "\t" << *x.first << "\n";
        }

    }
    template<typename Open_t, typename Prune_t = StandardNodeComp>
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
                std::cerr << "Goal found or expansion budget reached\n";
                //m.search_timer.stop();
                return std::make_pair(backup(cur, open_list).first, cur.g);
            }
            open_list.pop();
            open_list.fix_top();
            expand<Open_t, Prune_t>(g, cur, open_list, dest, m, hf);
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

