//
// Created by tonmoy on 02.01.19.
//

#ifndef ROSPERFORMACE_TEST_STATISTICS_H
#define ROSPERFORMACE_TEST_STATISTICS_H

#include <iostream>
#include <algorithm>

class Statistics {

public:
    double mean(double numbers[], double count);
    double standard_dev(double numbers[], double count);
    void meanprinter(double kuku);
    void stdvprinter(double kuku);
    void maxiprinter(double kuku);
    void miniprinter(double kuku);
    double max(double numbers[], int count);
    double mini(double numbers[], int count);

};


#endif //ROSPERFORMACE_TEST_STATISTICS_H
