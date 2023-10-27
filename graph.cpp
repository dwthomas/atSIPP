#include "graph.hpp"
#include "constants.hpp"
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

struct inATF{
    State source;
    State dest;
    EdgeATF eATF;
};

void read_ATF(std::istream& i, std::vector<inATF>& res){
    gIndex_t x, y;
    double st, en;
    std::string s;
    if(!(i >> x)){return;}
    i >> y;
    i >> s;
    st = stod(s);
    i >> s;
    en = stod(s);
    //std::cout << x << " ";
    //std::cout << y;
    State source(x, y, st, en);
    i >> x >> y; 
    i >> s;
    st = stod(s);
    i >> s;
    en = stod(s);
    //std::cout << " " << x << " " << y;
    State dest(x, y, st, en);
    intervalTime_t zeta, alpha, beta, delta;
    i >> s;
    //std::cout << source << " " << dest << " " << s << "\n";
    zeta = stod(s);
    i >> s;
    alpha = stod(s);
    i >> s;
    beta = stod(s);
    i >> s;
    delta = stod(s);
    //i >> zeta >> alpha >> beta >> delta;
    //std::cout << " " << zeta << " " << alpha << " " << beta << " " << delta << std::endl;
    EdgeATF edge(zeta, alpha, beta, delta);
    res.emplace_back(source, dest, edge); 
}

Graph read_graph(std::string filename){
    std::ifstream file(filename, std::ios_base::in | std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::gzip_decompressor());
    inbuf.push(file);
    //Convert streambuf to istream
    std::istream instream(&inbuf);
    //std::cout << instream.rdbuf();
 
    std::vector<inATF> res;
    while(!instream.eof()){
        read_ATF(instream, res);
    }
    file.close();
    // make GraphNodes
    Graph g;
    g.edges.reserve(res.size());
    for (const auto& entry: res){
        if (!g.nodes.contains(entry.source)){
            g.nodes[entry.source] = entry.source;
        }
        if (!g.nodes.contains(entry.dest)){
            g.nodes[entry.dest] = entry.dest;
        }
    }
    for (const auto & entry: res){ 
        g.edges.emplace_back(entry.eATF);
        g.edges.back().source = &g.nodes[entry.source];
        g.edges.back().destination = &g.nodes[entry.dest];
        g.nodes[entry.source].successors.emplace(&g.edges.back());
    }
    return g;
}

GraphNode&  find_earliest(const Graph& g, Location loc){
    GraphNode * cur = nullptr;
    for (auto& node: g.nodes){
        if ((cur == nullptr || begin(cur->state.interval) > begin(node.first.interval)) && loc == node.first.loc){
            cur = &node.second;
        }
    }
    if(cur == nullptr){
        std::cerr << "Unable to find starting vertex\n";
        exit(-1);
    }
    return *cur;
}