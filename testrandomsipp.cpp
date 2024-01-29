#include "randomsippgraph.hpp"
#include "map.hpp"

int main(){
    Map m("tiny.map");
    auto g = make_random_sipp_graph(m, 100, 0, 1, 20);
    std::cout << g << "\n";
}
