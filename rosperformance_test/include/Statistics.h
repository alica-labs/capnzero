#pragma once

#include <iostream>
#include <map>
template <typename x>
class Statistics {

public:
    x Onlinemean(x numbers[], x count);
    x mean(x numbers[], x count);
    x standard_dev(x numbers[], x count);
    void meanprinter(x  mEan);
    void stdvprinter(x  sTddv);
    void maxiprinter(x  mAx);
    void miniprinter(x  mIn);
    void Onlinemeanprinter(x  meAn);
    x  max(x  numbers[], int count);
    x  mini(x  numbers[], int count);
    void referencemean(std::map<long,double >& mYmap);
    void referencestd_dev(std::map<long,double >& mYmap);
    void rmax(std::map<long,double >& mYmap);
    void rmin(std::map<long,double >& mYmap);


};