#include <iostream>
#include <filesystem>
#include <ostream>
#include <boost/program_options.hpp>
#include "sipp.hpp"
#include "augmentedsipp.hpp"
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
        ("start,x", po::value<std::string>(), "starting location")
        ("goal,y", po::value<std::string>(), "goal location")
        ("edgegraph,g", po::value<std::filesystem::path>(),"gzip'd file containing the edge arrival time functions.")
        ("search,s", po::value<std::string>(), "Search algorithm to use")
        ("startTime,t", po::value<double>()->default_value(0.0), "Start Time of search.")
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
            Location source_loc(vm["start"].as<std::string>());
            Location goal_loc(vm["goal"].as<std::string>());
            std::cerr << "Reading graph\n";
            Graph g = read_graph(vm["edgegraph"].as<std::filesystem::path>().string());
            std::cerr << g << "\n";
            GraphNode * source = find_earliest(g,source_loc);
            if(vm["search"].as<std::string>() == "sipp"){
                MetaData m;
                double start_time = vm["startTime"].as<double>();
                auto res = sipp::search(source, goal_loc, m, start_time);
                for(auto n: res){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
            }
            else if(vm["search"].as<std::string>() == "asipp"){
                MetaData m;
                double start_time = vm["startTime"].as<double>();
                auto res = asipp::search(source, goal_loc, m, start_time);
                for(auto n: res.first){
                    std::cout << *n << "\n";
                }
                std::cout << res.second << "\n";
                std::cout << m << "\n";
            }

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
