#include <string>
#include <iostream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// SIPP Search algorithms 
#include "search_algorithms/SIPP/sipp.hpp"
#include "search_algorithms/augmentedSIPP/augmentedsipp.hpp"

// Any-start-time SIPP algorithms
#include "search_algorithms/RePEAT/repeat.hpp"

// Real-time SIPP algorithms
#include "search_algorithms/MinATFS/hybrid.hpp"
#include "search_algorithms/RTAS/rtasipp.hpp"
#include "search_algorithms/MaxATFS/plrtosipp.hpp"
#include "search_algorithms/PLRTS/plrtosipphonly.hpp"

inline void run_search(po::variables_map& vm, const Location& goal_loc, const SippGraph<Location>& g, const AtSippGraph<Location>& atg, double start_time, const SIPPState<Location> * source){
    MetaData m;
    if(vm["search"].as<std::string>() == "sipp"){
        auto res = sipp::search(g, source-&(g.vertices[0]), goal_loc, m, start_time);
        for(auto n: res){
            std::cout << *n << "\n";
        }
        std::cout << m << "\n";
    }
    else if(vm["search"].as<std::string>() == "asipp"){
        auto res = asipp::search(atg, source, goal_loc, m, start_time);
        for(auto n: res.first){
            std::cout << *n << "\n";
        }
        std::cout << m << "\n";
    }
    else if(vm["search"].as<std::string>() == "repeat"){
        auto res = rePEAT::search(atg, source, goal_loc, m, start_time);
        std::cout << m << "\n";
        std::cout << res;
    }
    else if(vm["search"].as<std::string>() == "rtas"){
        long budget = vm["budget"].as<long>();
        auto res = rtasipp::search(atg, source, goal_loc, m, start_time, budget);
        for(auto n: res){
            std::cout << *n << "\n";
        }
        std::cout << m << "\n";
    }
    else if(vm["search"].as<std::string>() == "maxatfs"){
        long budget = vm["budget"].as<long>();
        auto res = plrtosipp::search(atg, source, goal_loc, m, budget, start_time);
        for(auto n: res){
            std::cout << *n << "\n";
        }
        std::cout << m << "\n";
    }
    else if(vm["search"].as<std::string>() == "plrts"){
        long budget = vm["budget"].as<long>();
        auto res = plrtosipphonly::search(atg, source, goal_loc, m, budget, start_time);
        for(auto n: res){
            std::cout << *n << "\n";
        }
        std::cout << m << "\n";
    }
    else if(vm["search"].as<std::string>() == "medatfs"){
        long budget = vm["budget"].as<long>();
        auto res = hybrid::search(atg, source, goal_loc, m, budget, start_time);
        for(auto n: res){
            std::cout << *n << "\n";
        }
        std::cout << m << "\n";
    }
}