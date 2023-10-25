#include "constants.hpp"
#include <boost/container/flat_set.hpp>
#include <limits>

struct EdgeATF{
    intervalTime_t zeta;
    intervalTime_t alpha;
    intervalTime_t beta;
    intervalTime_t delta;
    std::vector<EdgeATF*> successors;

    EdgeATF(intervalTime_t _zeta, intervalTime_t _alpha, intervalTime_t _beta, intervalTime_t _delta):zeta(_zeta),alpha(_alpha),beta(_beta),delta(_delta){}

    inline intervalTime_t earliest_arrival_time() const{
        return alpha + delta;
    }

    inline intervalTime_t arrival_time(intervalTime_t t){
        if(t < zeta || beta <= t){
            return std::numeric_limits<intervalTime_t>::infinity();
        }
        if(t < std::min(alpha, beta)){
            return earliest_arrival_time();
        }
        return t + delta;
    }

    inline bool operator<(const EdgeATF& rhs) const{
        return earliest_arrival_time() < rhs.earliest_arrival_time();
    }
};

using EdgeATFList = boost::container::flat_set<EdgeATF>;
