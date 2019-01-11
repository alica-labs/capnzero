//
// Created by tonmoy on 02.01.19.
//
#include "Statistics.h"
#include <iostream>
#include <cmath>
double Statistics::Onlinemean(double numbers[], double count) {
    double caculated_mean=0.0;
    int n=0;
    double Xn = 0.0;
    double Xnminusone = 0.0;
    double post_mean=0.0;
    double Mean=0.0;
    for (int a=0;a < count; a++)
    {   n =+ 1;
        caculated_mean+=numbers[a];
        Xn=(caculated_mean/n);
        for (int a = 1; a <count ; ++a) {
            post_mean+=numbers[a];
            Xnminusone=(post_mean/a);
            Mean=Xn+(Xnminusone+((Xn+Xnminusone)/a));
        }
    }
    return Mean;
}
double Statistics::mean(double numbers[], double count) {
    double caculated_mean=0.0;

    double post_mean=0.0;
    for (int a=0;a < count; a++)
    {
        caculated_mean+=numbers[a];
    }
    caculated_mean/= double(count);
    return caculated_mean;
}
double Statistics::standard_dev(double numbers[], double count) {
    double st_dev=0.0;
    double average= mean(numbers,count);
    double temp_dev;
    for (int a=0;a < count; a++)
    {
        temp_dev=numbers[a]-average;
        st_dev +=temp_dev*temp_dev;
    }
    st_dev /=(count);
    st_dev = std::sqrt(st_dev);
    return st_dev;
}
void Statistics::meanprinter(double kuku) {
    std::cout<<"The mean of the elapsed time:"<<kuku<<std::endl;
}
void Statistics::stdvprinter(double kuku) {
    std::cout<<"The Standard deviation of the elapsed time:"<<kuku<<std::endl;
}
void Statistics::maxiprinter(double kuku) {
    std::cout<<"The max of the elapsed time:"<<kuku<<std::endl;
}
void Statistics::miniprinter(double kuku) {
    std::cout<<"The min of the elapsed time:"<<kuku<<std::endl;

}
void Statistics::Onlinemeanprinter(double Mean) {
    std::cout<<"The Online mean of the elapsed time:"<<Mean<<std::endl;
}
double Statistics::max(double numbers[], int count) {
    double maxi=numbers[0];

    for (int j = 1; j < (count); ++j)
    {
        if(numbers[j]>numbers[j-1])
        {
            maxi=numbers[j];
        }
        else
        {
            maxi=numbers[j-1];
        }

    }

    return maxi;
}
double Statistics::mini(double numbers[], int count) {
    double mini=numbers[0];
    for (int k = 1; k < (count); ++k) {

        if(mini>numbers[k])
        {
            mini=numbers[k];
        }


    }
    return mini;
}

