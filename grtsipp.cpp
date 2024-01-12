#include "grtsipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"

std::unordered_map<State, double> grtsipp::h;

inline void dump_h(){
    std::cerr << "h_static\n";
    for(auto acc: grtsipp::h){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
}

std::vector<GraphNode *> grtsipp::search(GraphNode * source, const Location& dest, MetaData & m, long budget, double start_time){
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
        std::cerr << "Searched. ";
        //learn static 
        m.learning_timer.resume();
        for (auto i : open_list.expanded){
            auto s = i.first->state;
            set_h(s, std::numeric_limits<double>::infinity());
        }
        DijkstraQueue dijkstra_open;
        for(auto n: open_list.queue){
            dijkstra_open.emplace(DijkstraNode(n.node, (n.f - n.g.earliest_arrival_time())));
        } 
        while(!dijkstra_open.empty() && !open_list.expanded.empty()){
            auto n = dijkstra_open.top();
            dijkstra_open.pop();
            if(open_list.expanded.contains(n.s)){
                open_list.expanded.erase(n.s);
            }
            double nh = get_h(*n.s, 0, dest);
            for (auto p: n.s->predecessors){
                auto s_p = p->source->state;
                double sph = get_h(*p->source, 0, dest);
                double delta = p->edge.delta;
                if(sph > delta + nh){
                    set_h(s_p, delta + nh);
                    dijkstra_open.emplace(DijkstraNode(p->source, delta + nh));
                }
            }
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
                std::cerr << "best TLA:" << *n << "\n";
                break;
            }
            n = nn;
        }
    } 
    return path;
}