#pragma once

#include <iostream>
#include <map>
class Statistics {

public:
    double Onlinemean(double numbers[], double count);
    double mean(double numbers[], double count);
    double standard_dev(double numbers[], double count);
    void meanprinter(double kuku);
    void stdvprinter(double kuku);
    void maxiprinter(double kuku);
    void miniprinter(double kuku);
    void Onlinemeanprinter(double Mean);
    double max(double numbers[], int count);
    double mini(double numbers[], int count);
    void refrencemean(std::map<long,double >& mYmap);
    void refrencestd_dev(std::map<long,double >& mYmap);
    void rmax(std::map<long,double >& mYmap);


};