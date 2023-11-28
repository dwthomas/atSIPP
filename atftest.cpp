#include <iostream>
#include "atf.hpp"

int main(){
    EdgeATF a(0, 0, 10, 5);
    EdgeATF b(0, 2, 7, 1);
    EdgeATF c(0, 7, 20, 10);
    std::vector<int*> n;
    CompoundATF catf(n);
    catf.add(a, n);
    catf.add(b, n);
    catf.add(c, n);
    std::cout << catf << "\n";
}