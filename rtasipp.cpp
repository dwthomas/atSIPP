#include <unordered_set>
#include "rtasipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

std::unordered_map<Location, double> rtasipp::h_static;
std::unordered_map<const GraphNode *, CompoundATF<GraphNode *>> rtasipp::h_dynamic;

inline void dump_h(){
    std::cerr << "h_static\n";
    for(auto acc: rtasipp::h_static){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
}

std::vector<GraphNode *> rtasipp::search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time){
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
        //dump_h();
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
        double h_s_prime = std::numeric_limits<double>::infinity();
        for (auto node: open_list.queue){
            double h_s_nx = get_h_s(*node.node, dest);
            h_s_prime = std::min(h_s_prime, node.g.delta + h_s_nx);
            GraphNode * frontiern = node.node;
            //std::cerr << new_path <<  " ";
            add_h_dyn(*cur, node.g, frontiern);
        }
        if(h_s_prime > get_h_s(*cur, dest)){
            //std::cerr << "stat: " << h_s_prime << " ";
            set_h_s(cur->state.loc, h_s_prime);
        }
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