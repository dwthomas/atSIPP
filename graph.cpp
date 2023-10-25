#include "graph.hpp"
#include "constants.hpp"
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

struct inATF{
    Location source;
    Location dest;
    EdgeATF eATF;
};

void read_ATF(std::istream& i, std::vector<inATF>& res){
    gIndex_t x, y;
    char buff[sizeof(double)]; 
    i.read(buff, sizeof(x));
    x = *(gIndex_t *)buff;
    i.read(buff, sizeof(x));
    y = *(gIndex_t *)buff;
    std::cout <<x << " " << y;
    Location source(x, y);
    i >> x >> y; 
    std::cout << " " << x << " " << y;
    Location dest(x, y);
    intervalTime_t zeta, alpha, beta, delta;
    i >> zeta >> alpha >> beta >> delta;
    std::cout << " " << zeta << " " << alpha << " " << beta << " " << delta << std::endl;
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
        g.edges.emplace_back(entry.eATF);
        g.edges.back().destination = &g.nodes[entry.dest];
        g.nodes[entry.source].successors.emplace(&g.edges.back());
    }
    return g;
}
