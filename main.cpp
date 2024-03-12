#include <iostream>
#include <filesystem>
#include <ostream>
#include <chrono>
#include <boost/program_options.hpp>
#include "sipp.hpp"
#include "augmentedsipp.hpp"
#include "repeat.hpp"
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
        ("agentSpeed,a", po::value<double>()->default_value(15.0), "Traveling speed of the agent.")
        ("walkingSpeed,w", po::value<double>()->default_value(1.0), "Walking speed for reversing train.")
        ("lookups,l", po::value<long>()->default_value(100), "Number of lookups to test repeat")
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
            double walkingSpeed(vm["walkingSpeed"].as<double>());
            double agentSpeed(vm["agentSpeed"].as<double>());
            std::cerr << "Reading graph\n";
            Graph g = read_graph(vm["edgegraph"].as<std::filesystem::path>().string(), agentSpeed, walkingSpeed);
            // std::cerr << g << "\n";
            // g.dump();
            
            // Check start and goal location exist in the graph
            bool foundStart = false;
            bool foundGoal = false;
            for (GraphNode n: g.node_array) {
                if (n.state.loc == source_loc) foundStart = true;
                if (n.state.loc == goal_loc) foundGoal = true;
            }
            if (!foundStart) std::cout << "[ERROR] Start location {" << source_loc.name << "} does not exist in graph\n";
            if (!foundGoal) std::cout << "[ERROR] Goal location {" << goal_loc.name << "} does not exist in graph\n";

            double start_time = vm["startTime"].as<double>();
            GraphNode * source = find_earliest(g, source_loc, start_time);
            if(vm["search"].as<std::string>() == "sipp"){
                MetaData m;
                auto res = sipp::search(source, goal_loc, m, start_time);
                //auto search_time = std::chrono::high_resolution_clock::now();
                //auto search_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(search_time - search_start_time);                                
                for(auto n: res){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
               // std::cout << "Search time: " << search_duration.count() << " nanoseconds\n";
            }
            else if(vm["search"].as<std::string>() == "asipp"){
                MetaData m;
                auto res = asipp::search(source, goal_loc, m, start_time);
                //auto search_time = std::chrono::high_resolution_clock::now();
                //auto search_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(search_time - search_start_time);
                for(auto n: res.first){
                    std::cout << *n << "\n";
                }
                std::cout << res.second << "\n";
                std::cout << m << "\n";
                //std::cout << "Search time: " << search_duration.count() << " nanoseconds\n";
            }
            else if(vm["search"].as<std::string>() == "repeat"){
                MetaData m;
                auto res = rePEAT::search(source, goal_loc, m, start_time);
                std::cout << m << "\n";
                std::cout << res;
              //  std::cout << "Search time: " << search_duration.count() << " nanoseconds\n";
                auto c = res.time_lookup(vm["lookups"].as<long>());
                double acc = 0.0;
                for (auto i : c){
                    acc += i;
                }
                std::cerr << acc << " " << c.size() << "\n";
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
