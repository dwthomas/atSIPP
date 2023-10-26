#include <iostream>
#include <filesystem>
#include <ostream>
#include <boost/program_options.hpp>
#include "structs.hpp"
#include "graph.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    try{
        // Declare options
        po::options_description desc("Allowed options");
        desc.add_options()
        ("help,h", "produce help message")
        //("map,m", po::value<std::filesystem::path>(), "Static map")
        ("startx,x", po::value<int>(), "x position of agent starting location")
        ("starty,y", po::value<int>(), "y position of agent starting location")
        ("goalx,X", po::value<int>(), "x position of goal location")
        ("goaly,Y", po::value<int>(), "y position of goal location")
        ("edgegraph,g", po::value<std::filesystem::path>(),"gzip'd file containing the edge arrival time functions.")
        ;
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);   
        // Parse options
        if (vm.count("help")){
            std::cout << desc << std::endl;
        }
        else if(vm.count("edgegraph") && std::filesystem::is_regular_file(vm["edgegraph"].as<std::filesystem::path>())){
            // read map
            std::cerr << "Reading graph\n";
            Graph g = read_graph("si.out.gz");
            std::cout << g << "\n";
        }
        else{
            std::cout << desc << std::endl;
        }
        
    }
    catch (const po::error &ex){
        std::cerr << ex.what() << std::endl;
    }
    return 0;
}
