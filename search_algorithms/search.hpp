#include "data_structures/atf.hpp"
#include <string>
#include <iostream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// SIPP Search algorithms 
#include "search_algorithms/SIPP/sipp.hpp"
#include "search_algorithms/augmentedSIPP/augmentedsipp.hpp"

// Augmented SIPP algorithms
#include "search_algorithms/aSIPPnew/augmentedsipp.hpp"
#include "search_algorithms/abSIPP/absipp.hpp"

// Any-start-time SIPP algorithms
#include "search_algorithms/RePEAT/repeat.hpp"

// Real-time SIPP algorithms
#include "search_algorithms/MedATFS/hybrid.hpp"
#include "search_algorithms/RTAS/rtasipp.hpp"
#include "search_algorithms/MaxATFS/plrtosipp.hpp"
#include "search_algorithms/PLRTS/plrtosipphonly.hpp"

inline void print_results(double arrival_time, std::vector<const SIPPState<Location> *> res, const MetaData& mdat){
    (void) res;
    std::cout << "\"arrival_time\": " << arrival_time << ",\n";
    std::cout << mdat;
}

inline void run_search(po::variables_map& vm, const Location& goal_loc, const SippGraph<Location>& g, const AtSippGraph<Location>& atg, double start_time, const SIPPState<Location> * source){
    MetaData m;
    if(vm["search"].as<std::string>() == "sipp"){
        auto search_res = sipp::search(g, source, goal_loc, m, start_time);
        auto& res = search_res.first;
        double arrival_time = search_res.second;
        std::cout << "\"results\": [\n{";
        print_results(arrival_time, res, m);
        std::cout << "\n}\n]";
    }
    else if(vm["search"].as<std::string>() == "asipp"){
        auto res = asipp::search(atg, source, goal_loc, m, start_time);
        std::cout << "\"results\": [\n{";
        print_results(res.second.earliest_arrival_time(), res.first, m);
        std::cout << "\n}\n]";
        CompoundATF<std::vector<const SIPPState<Location> *>> solutions;
        solutions.insert(res.second, res.first);
        std::cout << ",\n";
        std::cout << solutions;
    }
    else if(vm["search"].as<std::string>() == "augmentedsipp"){
        auto res = augmentedsipp::search(atg, source, goal_loc, m, start_time);
        std::cout << "\"results\": [\n{";
        print_results(res.second.earliest_arrival_time(), res.first, m);
        std::cout << "\n}\n]";
        CompoundATF<std::vector<const SIPPState<Location> *>> solutions;
        solutions.insert(res.second, res.first);
        std::cout << ",\n";
        std::cout << solutions;
    }
    else if(vm["search"].as<std::string>() == "absipp"){
        auto res = absipp::search(atg, source, goal_loc, m, start_time);
        std::cout << "\"results\": [\n{";
        print_results(res.second.earliest_arrival_time(), res.first, m);
        std::cout << "\n}\n]";
        CompoundATF<std::vector<const SIPPState<Location> *>> solutions;
        solutions.insert(res.second, res.first);
        std::cout << ",\n";
        std::cout << solutions;
    }
    else if(vm["search"].as<std::string>() == "repeat"){
        auto res = rePEAT::search(atg, source, goal_loc, m, start_time, vm["atlimit"].as<double>(), vm["test_query_time"].as<bool>());
        std::cout << res;
        auto& solutions = res.any_start_time_plan;
        std::cout << ",\n";
        std::cout << solutions;
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