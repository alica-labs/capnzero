#pragma once

#include <iostream>
#include <map>
template <typename x>
class Statistics {
public:
    Statistics();
    ~Statistics();

    double referencemean(std::map<long,double >& mYmap);
    double  referencestd_dev(std::map<long,double >& mYmap);
    double  rmax(std::map<long,double >& mYmap);
    double  rmin(std::map<long,double >& mYmap);
};