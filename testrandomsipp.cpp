#include "newatsippgraph.hpp"
#include "randomsippgraph.hpp"
#include "map.hpp"

int main(){
    Map m("tiny.map");
    auto g = make_random_sipp_graph(m, 100, 0, 1, 20, Location(0,0), Location(0,0));
    std::cout << g << "\n";
    AtSippGraph<Location> atg(&g);
    std::cout << atg << "\n"; 
}
