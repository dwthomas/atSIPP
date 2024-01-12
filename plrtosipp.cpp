#include "plrtosipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

std::unordered_map<Location, double> plrtosipp::h_static;
std::unordered_map<const GraphNode *, CompoundATF<GraphNode *>> plrtosipp::h_dynamic;

void plrtosipp::dump_h(){
    std::cerr << "dumping h_static\n";
    for(auto acc: plrtosipp::h_static){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
}

std::vector<GraphNode *> plrtosipp::search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<GraphNode *> path;
    CompoundATF solutions(path);
    m.init();
    auto cur = source;
    double t = start_time;
    m.search_timer.start();
    m.search_timer.stop();
    m.learning_timer.start();
    m.learning_timer.stop();
    while(!isGoal(*cur, dest)){
        //plrtosipp::dump_h();
        //search 
        std::cerr << *cur << " at " << t << " ";
        m.search_timer.resume();
        path.emplace_back(cur);
        Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), get_h(*cur, t, dest) , cur, nullptr);
        auto res = asipp::search_core(open_list, dest, m, budget, get_h);
        m.search_timer.stop();
        //std::cerr << "Searched. ";
        //learn
        m.learning_timer.resume();
        std::unordered_map<GraphNode *, double> open_h_s;
        std::unordered_map<GraphNode *, double> closed_copy(open_list.expanded);
        //asipp::dump_open(open_list);
        for(auto n: open_list.queue){
            open_h_s[n.node] = plrtosipp::get_h_s(*n.node, dest);
        }
        for (auto i : closed_copy){
            auto s = i.first->state;
            //std::cerr << "setting hs: " << s.loc << "\n";
            plrtosipp::set_h_s(s.loc, std::numeric_limits<double>::infinity());
            //std::cerr << plrtosipp::h_static.size() << "\n";
        }
        DijkstraQueue dijkstra_open;
        for(auto n: open_list.queue){
            dijkstra_open.emplace(DijkstraNode(n.node, (n.f - n.g.earliest_arrival_time())));
        } 
        while(!dijkstra_open.empty() && !closed_copy.empty()){
            auto n = dijkstra_open.top();
            dijkstra_open.pop();
            if(closed_copy.contains(n.s)){
                closed_copy.erase(n.s);
            }
            double nh;
            if(open_h_s.contains(n.s)){
                nh = open_h_s[n.s];
            }
            else{
                nh = plrtosipp::get_h_s(*n.s, dest);
            }
            for (auto p: n.s->predecessors){
                auto s_p = p->source->state;
                double sph = plrtosipp::get_h_s(*p->source, dest);
                double delta = p->edge.delta;
                if(sph > delta + nh){
                    plrtosipp::set_h_s(s_p.loc, delta + nh);
                    dijkstra_open.emplace(DijkstraNode(p->source, delta + nh));
                }
            }
        }
        // dynamic learning
        /*
        dijkstra_open.clear();
        closed_copy = open_list.expanded;
        for(auto n: open_list.queue){
            dijkstra_open.emplace(DijkstraNode(n.node, (n.f - n.g.earliest_arrival_time())));
            //plrtosipp::add_h_dyn(*n.node, shiftIdentity(0), n.node);
        } 
        while(!dijkstra_open.empty() && !closed_copy.empty()){
            auto n = dijkstra_open.top();
            dijkstra_open.pop();
            if(closed_copy.contains(n.s)){
                closed_copy.erase(n.s);
            }
            // double nh;
            // if(open_h_s.contains(n.s)){
            //     nh = open_h_s[n.s];
            // }
            // else{
            //     nh = plrtosipp::get_h_s(*n.s, dest);
            // }
            for (auto p: n.s->predecessors){
                auto s_p = p->source;
                auto compound_atf = plrtosipp::h_dynamic[s_p];
                double zeta = p->edge.zeta;

                for (std::size_t i = 1; i < compound_atf.edge_atfs.size(); i++){
                    auto path_atf = compound_atf.edge_atfs[i];
                    auto frontiern = compound_atf.payload[i];
                    double alpha = std::max(p->edge.alpha, path_atf.alpha - p->edge.delta);
                    double beta = std::min(p->edge.beta, path_atf.beta - p->edge.delta);
                    double delta = p->edge.delta + path_atf.delta;
                    //add_h_dyn(*s_p, EdgeATF(zeta, alpha, beta, delta), frontiern);
                }
                if(closed_copy.contains(s_p)){
                    dijkstra_open.emplace(DijkstraNode(s_p, plrtosipp::h_dynamic[s_p].earliest_arrival_time()));
                }
            }
        }*/
        m.learning_timer.stop();

        //std::cerr << "Learned. ";
        //commit
        auto n = open_list.top().node;
        if (open_list.queue.size() < 1){
            //asipp::dump_open(open_list);
            std::cout << "No path for agent!\n";
            exit(-1); 
        }
        while(open_list.parent[n] != nullptr){
            auto nn = open_list.parent[n];
            if (open_list.parent[nn] == nullptr){
                for (auto succ: cur->successors){
                    if(succ->destination == n){
                        t = succ->edge.arrival_time(t);
                        break;
                    }
                }
                cur = n; //best TLA
                //std::cerr << "best TLA:" << *n << "\n";
                break;
            }
            n = nn;
        }
    } 
    return path;
}