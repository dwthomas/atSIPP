#include <iostream>
#include <filesystem>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    try{
        // Declare options
        po::options_description desc("Allowed options");
        desc.add_options()
        ("help,h", "produce help message")
        ("map,m", po::value<std::filesystem::path>(), "Static map")
        ("graph,g", po::value<std::filesystem::path>(), "Search graph file")
        ("startx,x", po::value<int>(), "Starting location x coordinate")
        ("starty,y", po::value<int>(), "Starting location y coordinate")
        ("goalx,X", po::value<int>(), "Goal location x coordinate")
        ("goaly,Y", po::value<int>(), "Goal location y coordinate")
        ;
        po::positional_options_description pos_desc;
        //pos_desc.add("num", -1);

        po::command_line_parser parser{argc, argv};
        parser.options(desc).positional(pos_desc).allow_unregistered();
        po::parsed_options parsed_options = parser.run();

        po::variables_map vm;
        po::store(parsed_options, vm);
        po::notify(vm);


        // Parse options
        if (vm.count("help")){
            std::cout << desc << std::endl;
        }
        else if(vm.count("graph") && std::filesystem::is_regular_file(vm["graph"].as<std::filesystem::path>())){
         //else if(vm.count("map") && std::filesystem::is_regular_file(vm["map"].as<std::filesystem::path>()) && vm.count("graph") && std::filesystem::is_regular_file(vm["graph"].as<std::filesystem::path>())){  
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
