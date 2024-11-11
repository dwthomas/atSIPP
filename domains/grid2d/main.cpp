#include <iostream>
#include <filesystem>
#include <ostream>
#include <boost/program_options.hpp>
#include "data_structures/structs.hpp"
#include "domains/grid2d/map.hpp"
#include "search_algorithms/newatsippgraph.hpp"
#include "randomsippgraph.hpp"

#include "search_algorithms/search.hpp"
#include "search_algorithms/sippgraph.hpp"


namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    try{
        // Declare options
        po::options_description desc("Allowed options");
        desc.add_options()
        ("help,h", "produce help message")
        ("startx,x", po::value<int>()->default_value(-1), "x position of agent starting location")
        ("starty,y", po::value<int>()->default_value(-1), "y position of agent starting location")
        ("goalx,X", po::value<int>()->default_value(-1), "x position of goal location")
        ("goaly,Y", po::value<int>()->default_value(-1), "y position of goal location")
        ("map,m", po::value<std::filesystem::path>(),"static map file")
        ("scenario", po::value<std::filesystem::path>()->default_value("./"),"scenario file")
        ("search,s", po::value<std::string>(), "Search algorithm to use")
        ("startTime,t", po::value<double>()->default_value(0.0), "Start Time of search.")
        ("until,u", po::value<double>()->default_value(10.0), "Max time for random obstacles.")
        ("occupancy,o", po::value<double>()->default_value(0.0), "Occupancy of random obstacles")
        ("minDuration", po::value<double>()->default_value(1.0), "Min interval duration")
        ("maxDuration", po::value<double>()->default_value(10.0), "Max interval duration")
        ("budget,b", po::value<long>()->default_value(1), "Search budget in num of expansions")
        ("seed", po::value<long>()->default_value(0), "Random seed")
        ;
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);   
        // Parse options
        if (vm.count("help")){
            std::cout << desc << std::endl;
        }
        else if(vm.count("map") && std::filesystem::is_regular_file(vm["map"].as<std::filesystem::path>())){
            // read map
            //std::cerr << "Generating SIPP graph...";
            Map m(vm["map"].as<std::filesystem::path>().string());
            std::vector<Scenario> scenarios;
            if(std::filesystem::is_regular_file(vm["scenario"].as<std::filesystem::path>()) &&
                vm["scenario"].as<std::filesystem::path>().string() != ""){
                if(vm["startx"].as<int>() != -1 || vm["starty"].as<int>() != -1 || vm["goalx"].as<int>() != -1 || vm["goaly"].as<int>() != -1){
                    std::cerr << "Error: start and goal locations specified in scenario mode.\n";
                    exit(-1);
                }
                scenarios = Scenario::read_scenarios(vm["scenario"].as<std::filesystem::path>().string());
            }
            else{
                Location source_loc(vm["startx"].as<int>(), vm["starty"].as<int>());
                Location goal_loc(vm["goalx"].as<int>(), vm["goaly"].as<int>());
                scenarios.push_back(Scenario(source_loc, goal_loc));
            }
            for (const auto& scenario:scenarios){
                std::cerr << scenario << "\n";
                break;
            }
            double until = vm["until"].as<double>();
            double occupancy = vm["occupancy"].as<double>();
            double min_duration = vm["minDuration"].as<double>();
            double max_duration = vm["maxDuration"].as<double>();
            SippGraph<Location> g = make_random_sipp_graph(m, until, occupancy, min_duration, max_duration, vm["seed"].as<long>());
            //std::cout << g << "\n";
            //std::cerr << "SIPP graph made, compiling @SIPP graph...";
            for(const auto& scenario: scenarios){
                std::cerr << scenario << "\n";
                SippGraph<Location> copy_g(g);
                const SIPPState<Location> * source = mark_safe(copy_g, scenario._source);
                std::cerr << "Source: " << *source << "\n";
                mark_safe(copy_g, scenario._goal);
                std::cerr << "num verts: " << g.vertices.size() << "\n";
                AtSippGraph<Location> atg(&copy_g);
                std::cerr << "compiled!\n";
                // std::cerr << atg << "\n";
                //g.dump();
                double start_time = vm["startTime"].as<double>();
                // run search
                run_search(vm, scenario._goal, copy_g, atg, start_time, source);
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
