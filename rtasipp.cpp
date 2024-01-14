#include <unordered_set>
#include "rtasipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

std::unordered_map<Location, double> rtasipp::h_static;
std::unordered_map<const GraphNode *, rtasipp::CATF> rtasipp::h_dynamic;

inline void dump_h(){
    std::cerr << "h_static\n";
    for(auto acc: rtasipp::h_static){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
}

std::vector<GraphNode *> rtasipp::search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<GraphNode *> path;
    m.init();
    auto cur = source;
    double t = start_time;
    while(!isGoal(*cur, dest)){
        //search 
        //std::cerr << *cur << " at " << t << " ";
        m.search_timer.resume();
        // run NLASIPP
        path.emplace_back(cur);
        Open open_list;
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), get_h(*cur, t, dest) , cur, nullptr);
        auto res = asipp::search_core(open_list, dest, m, budget, get_h);
        // Done NLASIPP
        m.search_timer.stop();
        //learn
        m.learning_timer.resume();
        double h_s_prime = std::numeric_limits<double>::infinity();
        for (auto node: open_list.queue){
            double h_s_nx = get_h_s(*node.node, dest);
            h_s_prime = std::min(h_s_prime, node.g.delta + h_s_nx);
            add_h_dyn(*cur, &node.g);
        }
        if(h_s_prime > get_h_s(*cur, dest)){
            set_h_s(cur->state.loc, h_s_prime);
        }
        m.learning_timer.stop();

        if (open_list.queue.size() < 1){
            //asipp::dump_open(open_list);
            std::cout << "No path for agent!\n";
            exit(-1); 
        }
        auto n = open_list.top().node;
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