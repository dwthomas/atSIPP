#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <functional>
#include <limits>
#include <unordered_map>
#include "data_structures/atf.hpp"
#include "search_algorithms/newatsippgraph.hpp"
#include "search_algorithms/sippgraph.hpp"

namespace asipp{
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

    struct StandardNodeComp{
        inline bool operator()(const Node& a, const Node& b) const{
            if(a.f == b.f){
                return a.g < b.g;
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

    template <class NodeComp = StandardNodeComp>
    struct Open{
        using Queue = boost::heap::d_ary_heap<Node, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<NodeComp>>;
        typedef typename Queue::handle_type handle_t;

        Queue queue;
        std::unordered_map<const SIPPState<Location> *, const SIPPState<Location> *> parent;
        std::unordered_map<const SIPPState<Location> *, handle_t> handles;
        std::unordered_map<const SIPPState<Location> *, double> expanded;

        std::vector<SIPPState<Location>> extra_states;
        // std::vector<AtSIPPEdge<Location>> extra_edges;
        std::unordered_map<Location, SIPPState<Location> *> oth_expanded_states;    

        Open(){
            queue.reserve(n_prealloc());
            parent.reserve(n_prealloc());
            handles.reserve(n_prealloc());
            expanded.reserve(n_prealloc());
            extra_states.reserve(n_prealloc());
            // extra_edges.reserve(n_prealloc());
        }

        inline void emplace(EdgeATF e, double h, const SIPPState<Location> * n, const SIPPState<Location> * p, const AtSIPPEdge<Location> * tla){
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
            expanded[n.state] = n.g.earliest_arrival_time();
            queue.pop();
        }

        inline void decrease_key(handle_t handle , EdgeATF e, double h, const SIPPState<Location> * n, const SIPPState<Location> * p, const AtSIPPEdge<Location> * tla){
            parent[n] = p;
            queue.update(handle, Node(e, h, n, tla));
        }
    };

    template <typename Node_t>
    bool isGoal(const Node_t& n, const Location& goal_loc){
        return n.state->configuration == goal_loc;
    }


    template <typename Node_t, typename Open_t>
    std::pair<std::vector<const SIPPState<Location> *>, double> backup(const Node_t& n, Open_t& open_list){
        std::vector<const SIPPState<Location> *> res;
        const SIPPState<Location>* cur = n.state;
        while(cur != nullptr){
            res.push_back(cur);
            cur = open_list.parent[cur];
        }
        std::reverse(res.begin(), res.end());
        //std::cout << "Arrival time: " << n.f << " " << n << "\n";
        return std::make_pair(res, n.f);
    }

    template <typename Node_t, typename Open_t>
    inline void expand(const AtSippGraph<Location>& g, const Node_t& cur, Open_t& open_list, const Location& goal_loc, MetaData & m, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        m.expanded++;
        double zeta = cur.g.zeta;
        //std::cerr << cur.state << *cur.state << "\n";
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
            else if (open_list.handles.contains(successor.destination)){
                auto handle = open_list.handles[successor.destination];
                if(arrival_time_function.earliest_arrival_time() < (*handle).g.earliest_arrival_time()){
                    m.decreased++;
                    double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                    //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                    open_list.decrease_key(handle ,arrival_time_function, h, successor.destination, successor.source, tla);
                }
            }
            else{
                m.generated++;
                double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                open_list.emplace(arrival_time_function, h, successor.destination, successor.source, tla);
                //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
            }
        }
    }

    template <typename Node_t, typename Open_t>
    inline void expand_th(const AtSippGraph<Location>& g, const Node_t& cur, Open_t& open_list, const Location& goal_loc, double time_horizon, MetaData & m, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        m.expanded++;
        double zeta = cur.g.zeta;
        //std::cerr << cur.state << *cur.state << "\n";
        for(const AtSIPPEdge<Location>& successor: g.successors.at(cur.state)){ // add another successor in each direction & remove the rest past time horizon
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
            if(arrival_time_function.earliest_arrival_time() >= time_horizon){
                m.time_horizon_hits++;
                // if(m.time_horizon_hits % 1000 == 0) 
                //     std::cerr << "hit " << m.time_horizon_hits << "\n";
                continue;
            }
            if(open_list.expanded.contains(successor.destination)){
                continue;
            }
            else if (open_list.handles.contains(successor.destination)){
                auto handle = open_list.handles[successor.destination];
                if(arrival_time_function.earliest_arrival_time() < (*handle).g.earliest_arrival_time()){
                    m.decreased++;
                    double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                    //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                    open_list.decrease_key(handle ,arrival_time_function, h, successor.destination, successor.source, tla);
                }
            }
            else{
                m.generated++;
                double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                open_list.emplace(arrival_time_function, h, successor.destination, successor.source, tla);
                //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
            }
        }
    }

    template <typename Node_t, typename Open_t>
    inline void expand_oth(const AtSippGraph<Location>& g, const Node_t& cur, Open_t& open_list, const Location& goal_loc, double time_horizon, MetaData & m, std::unordered_map<const SIPPState<Location> *, std::vector<AtSIPPEdge<Location>>> & extra_successors, 
                                                                                        std::unordered_map<const SIPPState<Location> *, std::vector<AtSIPPEdge<Location>>> & extra_predecessors, 
                                                                                          double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        m.expanded++;
        double zeta = cur.g.zeta;
        //std::cerr << cur.state << *cur.state << "\n";
        std::vector<const AtSIPPEdge<Location> *> succ;
        if( g.successors.contains(cur.state) ){
            for (const AtSIPPEdge<Location>& s : g.successors.at(cur.state)){
                // auto tla = cur.tla;
                // if (tla == nullptr){
                //     tla = &s;
                // }
                double alpha = std::max(cur.g.alpha, s.duration.alpha - cur.g.delta);
                double beta = std::min(cur.g.beta, s.duration.beta - cur.g.delta);
                double delta = s.duration.delta + cur.g.delta;
                EdgeATF arrival_time_function(zeta, alpha, beta, delta);
                if(arrival_time_function.earliest_arrival_time() < time_horizon){
                    succ.push_back(&s);
                }
            }
        }

        if(cur.g.earliest_arrival_time() + 1 >= time_horizon){
            for (int dx : {-1, 0, 1}){
                for (int dy : {-1, 0, 1}){
                    if (dx == 0 && dy == 0) continue;
                    Location neighbor_loc = {cur.state->configuration.x() + dx, cur.state->configuration.y() + dy};
                    if(!g.sipp_graph->map.inBounds(neighbor_loc.x(), neighbor_loc.y()) || !g.sipp_graph->map.isSafe(neighbor_loc.x(), neighbor_loc.y())) continue;
                    SIPPState<Location>* neighbor_state;
                    if (open_list.oth_expanded_states.contains(neighbor_loc)){
                        neighbor_state = open_list.oth_expanded_states[neighbor_loc];
                    }
                    else{
                        open_list.extra_states.emplace_back(neighbor_loc, cur.state->safe_interval);
                        neighbor_state = &open_list.extra_states.back();
                        open_list.oth_expanded_states[neighbor_loc] = neighbor_state;
                    }
                    EdgeATF arrival_time_function(cur.g.zeta, 0, std::numeric_limits<double>::infinity(), 1);
                    if(!extra_successors.contains(cur.state)){
                        extra_successors[cur.state].reserve(1000);
                    }
                    extra_successors[cur.state].emplace_back(cur.state, neighbor_state, arrival_time_function);
                    if(!extra_successors.contains(cur.state)){
                        extra_successors[cur.state].reserve(1000);
                    }
                    extra_predecessors[neighbor_state].emplace_back(neighbor_state, cur.state, arrival_time_function);
                    // open_list.extra_edges.emplace_back(cur.state, neighbor_state, arrival_time_function);
                    
                    const AtSIPPEdge<Location>& new_edge = extra_successors[cur.state].back();
                    succ.emplace_back(&new_edge);
                    open_list.oth_expanded_states[neighbor_loc] = neighbor_state;
                }
            }
        }

        for(const AtSIPPEdge<Location>* s: succ){ // add another successor in each direction & remove the rest past time horizon
            const auto& successor = *s;
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
            // if(arrival_time_function.earliest_arrival_time() >= time_horizon){
            //     m.time_horizon_hits++;
            //     // if(m.time_horizon_hits % 1000 == 0) 
            //     //     std::cerr << "hit " << m.time_horizon_hits << "\n";
            //     continue;
            // }
            if(open_list.expanded.contains(successor.destination)){
                continue;
            }
            else if (open_list.handles.contains(successor.destination)){
                auto handle = open_list.handles[successor.destination];
                if(arrival_time_function.earliest_arrival_time() < (*handle).g.earliest_arrival_time()){
                    m.decreased++;
                    double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                    //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                    open_list.decrease_key(handle ,arrival_time_function, h, successor.destination, successor.source, tla);
                }
            }
            else{
                m.generated++;
                double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                //double h = eightWayDistance(successor->destination->state.loc, goal_loc);
                open_list.emplace(arrival_time_function, h, successor.destination, successor.source, tla);
                //std::cerr << "Generated: " << *successor  << " from: " << *successor->source << " to: " << *successor->destination  << "\n";
            }
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
        for( auto x : open_list.expanded){
            std::cerr <<  "\t" << *x.first << "\n";
        }

    }
    template<typename Open_t>
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


    template<typename Open_t>
    inline std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> search_th_core(const AtSippGraph<Location>& g, Open_t& open_list, const Location& dest, MetaData & m, long expansion_budget = -1, double time_horizon = std::numeric_limits<double>::infinity(), double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
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
            expand_th(g, cur, open_list, dest, time_horizon, m, hf);
            if (open_list.empty()){
                open_list.emplace(cur.g, cur.f - cur.g.earliest_arrival_time(), cur.state, open_list.parent[cur.state], cur.tla);
                // open_list.push(cur);
                break;
            }
        }
        //std::cerr << "Failed to find path\n";
        return std::make_pair(std::vector<const SIPPState<Location> *>(), EdgeATF());
        // std::cerr << "Failed to find path\n";
        // exit(-1);
    }

     template<typename Open_t>
    inline std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> search_oth_core(const AtSippGraph<Location>& g, Open_t& open_list, const Location& dest, MetaData & m, 
                                                                                        std::unordered_map<const SIPPState<Location> *, std::vector<AtSIPPEdge<Location>>> & extra_successors, 
                                                                                        std::unordered_map<const SIPPState<Location> *, std::vector<AtSIPPEdge<Location>>> & extra_predecessors,  
                                                                                        long expansion_budget = -1, double time_horizon = std::numeric_limits<double>::infinity(), double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper ){
        long start_expansions = m.expanded;
        //m.search_timer.start();
        while(!open_list.empty()){
           //dump_open(open_list);
            auto cur = open_list.top();
            // std::cout << cur << "\n";
            // if (cur.tla != nullptr){
            //     std::cerr << "tla: " << "\n"; 
            //     std::cerr << (cur.tla) << "\n";
            //     std::cerr << *(cur.tla) << "\n";
            // }
           // std::cerr << expansion_budget << "\n";
            //std::cerr << isGoal(cur, dest) << " " << (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget) << "\n";
            if(isGoal(cur, dest) || (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget)){
                //m.search_timer.stop();
                return std::make_pair(backup(cur, open_list).first, cur.g);
            }
            open_list.pop();
            expand_oth(g, cur, open_list, dest, time_horizon, m, extra_successors, extra_predecessors, hf);
            if (open_list.empty()){
                // std::cerr << "Re-adding best node to open\n";
                // std::cerr << "Node: " << cur << "\n";
                open_list.emplace(cur.g, cur.f - cur.g.earliest_arrival_time(), cur.state, open_list.parent[cur.state], cur.tla);
                // open_list.push(cur);
                break;
            }
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

