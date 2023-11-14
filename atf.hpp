#pragma once
#include "constants.hpp"
#include <boost/container/flat_set.hpp>
#include <vector>
#include <set>
#include <limits>
#include <format>

struct EdgeATF;

struct ATFSegment{
    double x0;
    double x1;
    double y0;
    double y1;
    long parent;
    ATFSegment() = default;
    ATFSegment(double b, double e, double s, double a, long p):x0(b),x1(e),y0(s),y1(a),parent(p){}

    inline bool operator<(const ATFSegment& seg) const{
        return x0 < seg.x0;
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const ATFSegment& seg){
        stream << "<" << seg.x0 << "," << seg.x1 << "," << seg.y0 << "," << seg.y1 << ">";
        return stream;
    }
};

struct EdgeATF{
    intervalTime_t zeta;
    intervalTime_t alpha;
    intervalTime_t beta;
    intervalTime_t delta;
    std::vector<EdgeATF*> successors;
    EdgeATF() = default;
    EdgeATF(intervalTime_t _zeta, intervalTime_t _alpha, intervalTime_t _beta, intervalTime_t _delta):zeta(_zeta),alpha(_alpha),beta(_beta),delta(_delta){}

    inline intervalTime_t earliest_arrival_time() const{
        return alpha + delta;
    }

    inline intervalTime_t arrival_time(intervalTime_t t) const{
        if(t < zeta || beta <= t){
            return std::numeric_limits<intervalTime_t>::infinity();
        }
        if(t < std::min(alpha, beta)){
            return earliest_arrival_time();
        }
        return t + delta;
    }

    inline intervalTime_t inclusive_arrival_time(intervalTime_t t) const{
        if(t < zeta || beta < t){
            return std::numeric_limits<intervalTime_t>::infinity();
        }
        if(t < std::min(alpha, beta)){
            return earliest_arrival_time();
        }
        return t + delta;
    }

    inline intervalTime_t supremum_arrival_time() const{
        return beta + delta;
    }

    inline bool operator<(const EdgeATF& rhs) const{
        return earliest_arrival_time() < rhs.earliest_arrival_time();
    }
    
    inline friend std::ostream& operator<< (std::ostream& stream, const EdgeATF& eatf){
        stream << "<" << eatf.zeta << "," << eatf.alpha << "," << eatf.beta << "," << eatf.delta << ">";
        return stream;
    }

    inline std::pair<ATFSegment, ATFSegment> segments() const{
        std::pair<ATFSegment, ATFSegment> res;
        double periapsis = arrival_time(alpha);
        double apoapsis = arrival_time(beta);
        res.first = ATFSegment(zeta, alpha, periapsis, periapsis, -1);
        res.second = ATFSegment(alpha, beta, periapsis, apoapsis, -1);
        return res;
    }
};

using EdgeATFList = boost::container::flat_set<EdgeATF>;

struct CompoundATF{
    std::vector<EdgeATF> edge_atfs;
    std::set<ATFSegment> segments;

    CompoundATF(){
        edge_atfs.emplace_back(
            -std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()
        );
        segments.emplace(
            -std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            0.0,
            std::numeric_limits<double>::infinity(),
            0);
    }

    inline void add(const EdgeATF& e){
        edge_atfs.emplace_back(e);
        auto segments = e.segments();
        segments.first.parent = edge_atfs.size()-1;
        segments.second.parent = edge_atfs.size()-1;
        (void)segments;
    }
};
