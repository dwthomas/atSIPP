#include "propeat.hpp"
#include "../RePEAT/updateRefTime.hpp"
#include "data_structures/atf.hpp"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <time.h>


proPEAT::Results proPEAT::search(const AtSippGraph<Location> &g,
                                 const SIPPState<Location> *source,
                                 const Location &dest, MetaData &mdat,
                                 double start_time, double end_time,
                                 bool test_query_time) {
  double t_ref = start_time;
  std::vector<const SIPPState<Location> *> path;
  // CompoundATF<std::vector<const SIPPState<Location> *>> solutions;
  proPEAT::Results res;
  auto &solutions = res.any_start_time_plan;
  struct timespec ts1, ts2;

  augmentedsipp::Open<augmentedsipp::ABNodeComp> open_list;
  EdgeATF init =
      EdgeATF(t_ref, t_ref, std::numeric_limits<double>::infinity(), 0.0);
  open_list.emplace(init, eightWayDistance(dest, source->configuration), source,
                    0, nullptr);

  while (t_ref < end_time) {
    // std::cerr << "tref: " << t_ref << "\n";
    res.reference_times.push_back(t_ref);
    double old_reference_time = t_ref;
    auto &m = res.search_metadata.emplace_back();
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    auto res = augmentedsipp::search_core<
        augmentedsipp::Open<augmentedsipp::ABNodeComp>,
        augmentedsipp::DominancePrune>(g, open_list, dest, m);
    if (res.second.beta > 0) {
      solutions.insert(res.second, res.first);
      t_ref = update_reference_time(res.second, open_list);
    } else {
      t_ref = end(source->safe_interval);
    }

    open_list.reorder_open_list( t_ref, old_reference_time);

    // std::cerr << "tref: " << t_ref << " " << end(source->safe_interval)
    // <<"\n";
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    m.search_time = 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec -
                    (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
  }

  // test query time
  if (test_query_time) {
    long n = 1000000000;
    struct timespec ts3, ts4;
    std::vector<double> t_depart;
    boost::random::mt19937 gen;
    boost::random::uniform_real_distribution<> dis(0.0, end_time);
    for (long i = 0; i < n; i++) {
      double sample = dis(gen);
      t_depart.push_back(sample);
    }
    double sum_arrival_time = 0.0;
    clock_gettime(CLOCK_MONOTONIC, &ts3);

    for (auto td : t_depart) {
      sum_arrival_time += solutions.arrival_time(td);
    }
    clock_gettime(CLOCK_MONOTONIC, &ts4);
    std::cerr << "total arrival time: " << sum_arrival_time << "\n";
    double query_time = 1000.0 * ts4.tv_sec + 1e-6 * ts4.tv_nsec -
                        (1000.0 * ts3.tv_sec + 1e-6 * ts3.tv_nsec);
    std::cerr << "\"Average query time:\" " << query_time / n << " ms,\n";
  }

  return res;
}