#include "segment.hpp"
#include <iostream>


int main(){
    std::cout << "Both Flat:\n";
    for(double x = -2; x <= 1.0; x+=0.5){
        Segment b(0, 1, 1, 1, -1);
        Segment a(x, x+2, 2, 2, -1);

        std::cout << "a: " << a << ", b: " << b << "\n";
        auto res = lowerHull(a, b);
        std::cout << " LowerHull(a, b): ";
        for(auto i : res){
            std::cout << i << " ";
        }
        std::cout << "\n";
        auto res1 = lowerHull(b, a);
        for(std::size_t i = 0; i < res.size(); i++){
           assert(res[i] == res1[i]); 
        }
        std::cout << "\n";
    }
    std::cout << "Both Rising\n";
    for(double x = -2; x <= 1.0; x+=0.5){
        Segment b(0, 1, 1, 2, -1);
        Segment a(x, x+2, 1, 3, -1);

        std::cout << "a: " << a << ", b: " << b << "\n";
        auto res = lowerHull(a, b);
        std::cout << " LowerHull(a, b): ";
        for(auto i : res){
            std::cout << i << " ";
        }
        std::cout << "\n";
        auto res1 = lowerHull(b, a);
        for(std::size_t i = 0; i < res.size(); i++){
           assert(res[i] == res1[i]); 
        }
        std::cout << "\n";
    }
    std::cout << "B Rising\n";
    for(double x = -2; x <= 1.0; x+=0.5){
        Segment b(0, 1, 1.5, 2.5, -1);
        Segment a(x, x+2, 2, 2, -1);

        std::cout << "a: " << a << ", b: " << b << "\n";
        auto res = lowerHull(a, b);
        std::cout << " LowerHull(a, b): ";
        for(auto i : res){
            std::cout << i << " ";
        }
        std::cout << "\n";
        auto res1 = lowerHull(b, a);
        for(std::size_t i = 0; i < res.size(); i++){
           assert(res[i] == res1[i]); 
        }
        std::cout << "\n";
    }
    std::cout << "A Rising\n";
    for(double x = -2; x <= 1.0; x+=0.5){
        Segment b(0, 1, 2, 2, -1);
        Segment a(x, x+2, 1, 3, -1);

        std::cout << "a: " << a << ", b: " << b << "\n";
        auto res = lowerHull(a, b);
        std::cout << " LowerHull(a, b): ";
        for(auto i : res){
            std::cout << i << " ";
        }
        std::cout << "\n";
        auto res1 = lowerHull(b, a);
        for(std::size_t i = 0; i < res.size(); i++){
           assert(res[i] == res1[i]); 
        }
        std::cout << "\n";
    }

    Segment b(0, 2, 1, 3, -1);
    Segment a(-0.5, 1.5, 2, 2, -1);

    std::cout << "a: " << a << ", b: " << b << "\n";
    auto res = lowerHull(a, b);
    std::cout << " LowerHull(a, b): ";
    for(auto i : res){
        std::cout << i << " ";
    }
    std::cout << "\n";
    auto res1 = lowerHull(b, a);
    for(std::size_t i = 0; i < res.size(); i++){
        assert(res[i] == res1[i]); 
    }
    std::cout << "\n";
}