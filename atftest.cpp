#include <iostream>
#include "atf.hpp"

int main(){
    EdgeATF a(0, 0, 10, 5);
    EdgeATF b(0, 2, 7, 1);
    EdgeATF c(0, 7, 20, 10);
    CompoundATF catf;
    catf.add(a);
    catf.add(b);
    catf.add(c);
    std::cout << catf << "\n";
}