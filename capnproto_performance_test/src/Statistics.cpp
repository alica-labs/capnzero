//
// Created by tonmoy on 02.01.19.
//
#include "Statistics.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

template <typename x>
 Statistics<x>::Statistics() {

}

template <typename x>
Statistics<x>::~Statistics() {

}
template <typename x>
double Statistics<x>::referencemean(std::map<long, double> &mYmap) {
    x mean = 0;
    int n=0;
    for (auto it = mYmap.begin();
         it != mYmap.end(); ++it){
        n += 1;
        mean = mean + (((it->second) - mean) /n); // As I am using my dividend in double format and divisor in int ,the quotient will be double .
    }// source-https://stackoverflow.com/questions/7571326/why-does-dividing-two-int-not-yield-the-right-value-when-assigned-to-double
std::cout<< "Mean from our STL map library: "<<mean<<" ns"<<std::endl;
    return mean;
}
template <typename x>
double  Statistics<x>::referencestd_dev(std::map<long, double> &mYmap) {
    x mean = 0;
    auto st_dev=0.0;
    x temp_dev=0;
    int n=0;
    for (auto it = mYmap.begin();it != mYmap.end(); ++it){
        n += 1;
        mean = mean + (((it->second) - mean) /n);
    }for (auto i = mYmap.begin(); i!= mYmap.end() ; ++i){
        temp_dev=((i->second)-mean);
          st_dev += temp_dev* temp_dev;
    }
    st_dev /=(n);
    st_dev = std::sqrt(st_dev);
    std::cout<< "Std_dev from our STL map library: "<<st_dev<<" ns"<<std::endl;
    return st_dev;
}

template <typename x>
double  Statistics<x>::rmax(std::map<long, double> &mYmap) {
    std::vector<x> v;

    for (auto it = mYmap.begin(); it != mYmap.end(); ++it) {
        v.push_back(it->second);

    }
    auto maxV=*max_element(v.begin(),v.end());
    std::cout << "The maximum value from our STL map  library: " << maxV <<" ns"<<std::endl;
    return maxV;
}
template <typename x>
double  Statistics<x>::rmin(std::map<long, double> &mYmap) {
    std::vector<x> v;
    for (auto it = mYmap.begin(); it != mYmap.end(); ++it)
    {
        v.push_back(it->second);
    }
    auto minV=*min_element(v.begin(),v.end());
    std::cout << "The Minimum value from our STL map  library: " << minV <<" ns"<<std::endl;
    return minV;
}

template class Statistics<double>;

