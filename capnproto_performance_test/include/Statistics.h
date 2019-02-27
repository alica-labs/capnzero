#pragma once

#include <iostream>
#include <map>
template <typename x>
class Statistics {
public:
    Statistics();
    ~Statistics();
    void referencemean(std::map<long,double >& mYmap);
    void referencestd_dev(std::map<long,double >& mYmap);
    void rmax(std::map<long,double >& mYmap);
    void rmin(std::map<long,double >& mYmap);
};