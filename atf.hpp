#pragma once
#include <boost/container/flat_set.hpp>
#include <vector>
#include <set>
#include <limits>
#include <format>

#include "constants.hpp"
#include "segment.hpp"
#include <iostream>

struct EdgeATF;

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

    inline segments_small_container segments() const{
        segments_small_container res;
        double periapsis = arrival_time(alpha);
        double apoapsis = inclusive_arrival_time(beta);
        if(alpha > zeta){
            res.emplace_back(zeta, alpha, periapsis, periapsis, -1);
        }
        if(beta > alpha){
            res.emplace_back(alpha, beta, periapsis, apoapsis, -1);
        }
        return res;
    }
};

using EdgeATFList = boost::container::flat_set<EdgeATF>;

template <typename T>
struct CompoundATF{
    std::vector<EdgeATF> edge_atfs;
    std::vector<T> payload;
    std::set<Segment> segments;

    CompoundATF(const T& init){
        edge_atfs.emplace_back(
            0,
            0,
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()
        );
        payload.emplace_back(init);
        segments.emplace(
            0.0,
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            0);
    }

    inline bool monotonic_non_decreasing() const{
        if (segments.size() < 2){
            return true;
        }
        bool res = true;
        auto a = segments.begin();
        auto b = std::next(a);
        while(b != segments.end()){
            res = res && a->y1 <= b->y0; 
            a = b;
            b++;
        }
        return res;
    } 

    inline bool bumper_to_bumper() const{
        if (segments.size() < 2){
            return true;
        }
        bool res = true;
        auto a = segments.begin();
        auto b = std::next(a);
        while(b != segments.end()){
            res = res && a->x1 == b->x0; 
            a = b;
            b++;
        }
        return res;
    }    

    inline void add_segment(const Segment& segment){
        Segment seg = segment;
        //std::cerr << "Adding: " << seg << "\n";
        auto it = segments.lower_bound(segment);
        while(true){
            if(!overlap(seg, *it)){
                segments.emplace_hint(it, seg);
                break;
            }
            auto hull = lowerHull(*it, seg);
            //std::cerr << "lowerhull: " << *it << ", " << seg << " is: ";
            //for(int i = 0; i < hull.size(); i++){
            //    std::cerr << hull[i] << ", ";
            //}
            //std::cerr << "\n";
            seg = hull[0];
            //std::cerr << "deleting: " << *it << "\n";
            it = segments.erase(it);
            for(int i = hull.size()-1; i > 0; i--){
                //std::cerr << "placing: " << hull[i] << "\n";
                it = segments.emplace_hint(it, hull[i]);
            }
            std::cout << "getting previous " << *it << "\n";
            if (it == segments.begin()) {
                std::cerr << "Already at begin of iterator.\n";
                std::cerr << "placing: " << seg << " (segment)\n";
                segments.emplace_hint(it, seg);
                break;
            }
            it = std::prev(it);
            if(it == segments.begin()){
                std::cerr << "placing: " << seg << " (segment)\n";
                segments.emplace_hint(it, seg);
                break;
            }
        }        
    }

    inline void add(const EdgeATF& e, const T& p){
        //std::cerr << "atf: " << e << "\n";
        edge_atfs.emplace_back(e);
        payload.emplace_back(p);
        auto segments = e.segments();
        for(auto segment: segments){
            segment.payload = edge_atfs.size()-1;
            add_segment(segment);
        }
        //std::cerr << "cATF: "<< *this << "\n";
        assert(bumper_to_bumper());
        assert(monotonic_non_decreasing());
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const CompoundATF& catf){
        for(const auto& segment : catf.segments){
            stream << segment << ", ";
        }
        stream << "\n";
        for(const auto& segment : catf.segments){
            for (auto j : catf.payload[segment.payload]){
                stream << *j << "\n";
            }
            stream << catf.edge_atfs[segment.payload] << "\n";
        }
        return stream;
    }
};
