#include <iostream>
#include "atf.hpp"

int main(){
    EdgeATF a(0, 0, 10, 5);
    EdgeATF b(0, 2, 7, 1);
    EdgeATF c(0, 7, 20, 10);
    CompoundATF<void *> catf;
    catf.insert(&a, nullptr);
    catf.insert(&b, nullptr);
    catf.insert(&c, nullptr);
    std::cout << catf << "\n";
}