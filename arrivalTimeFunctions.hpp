#include "constants.hpp"
#include "structs.hpp"

struct EdgeATF{
    intervalTime_t zeta, alpha, beta, delta;
    EdgeATF(intervalTime_t _zeta, intervalTime_t _alpha, intervalTime_t _beta, intervalTime_t _delta):zeta(_zeta),alpha(_alpha),beta(_beta),delta(_delta){};

    inline intervalTime_t earliest_arrival_time() const{
        return alpha + delta;
    }

    inline intervalTime_t supremum_arrival_time() const{
        return beta + delta;
    }

    inline intervalTime_t arrival_time(intervalTime_t t) const{
        if(t < zeta || t >= beta){
            return std::numeric_limits<intervalTime_t>::infinity();
        }
        if(t <= alpha){
            return earliest_arrival_time();
        }
        return t + delta;
    }
};