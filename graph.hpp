#include "structs.hpp"
#include "atf.hpp"
#include <unordered_map>
#include <boost/container/flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

struct GraphEdge;
struct GraphNode;

struct GraphNode{
    Location loc;
    boost::container::flat_set<GraphEdge *> successors;
    GraphNode() = default;
    GraphNode(const Location& l):loc(l){}
    inline friend std::ostream& operator<< (std::ostream& stream, const GraphNode& gn){
        stream << gn.loc << " ns:" << gn.successors.size();
        return stream;
    }
};

struct GraphEdge{
    EdgeATF edge;
    GraphNode * destination;
    GraphEdge(const EdgeATF& e):edge(e){
        destination = nullptr;
    }
    inline friend std::ostream& operator<< (std::ostream& stream, const GraphEdge& ge){
        stream << ge.edge << "->" << *ge.destination;
        return stream;
    }
};

struct Graph{
    std::vector<GraphEdge> edges;
    boost::unordered::unordered_flat_map<Location, GraphNode, std::hash<Location>> nodes;
    Graph() = default;
    inline void dump() const{
        for (const auto & n: nodes){
            std::cout << n.second;
            std::cout << "succ: ";
            for (const auto& s: n.second.successors){
                std::cout << s ;
            }
            std::cout << "\n";
        }
        for (const auto& e: edges){
            std::cout << e << "\n";
        }
    }
    inline friend std::ostream& operator<< (std::ostream& stream, const Graph& g){
        stream << g.edges.size() << " edges, " << g.nodes.size() << " nodes";       
        return stream;
    }
};

Graph read_graph(std::string filename);
