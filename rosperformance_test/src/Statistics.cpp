//
// Created by tonmoy on 02.01.19.
//
#include "Statistics.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
void Statistics::refrencemean(std::map<long, double> &mYmap) {
    double mean = 0;
    int n=0;
    for (auto it = mYmap.begin();
         it != mYmap.end(); ++it){
        n += 1;
        mean = mean + (((it->second) - mean) /n);
    }
std::cout<< "Mean from our STL map library "<<mean<<std::endl;
}

void Statistics::refrencestd_dev(std::map<long, double> &mYmap) {
    double mean = 0;
    auto st_dev=0.0;
    double temp_dev=0;
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
    std::cout<< "Std_dev from our STL map library "<<st_dev<<std::endl;
}


void Statistics::rmax(std::map<long, double> &mYmap) {
    std::vector<double> v;
    double maxi = 0.0;
    int n = 0;
    //v[] = {0};
    for (auto it = mYmap.begin(); it != mYmap.end(); ++it) {
        v.push_back(it->second);

    }
    for (int i=0; i <v.size(); ++i){
        if (v[i] > v[(i - 1)]) {
            maxi = v[i];
        } else {
            maxi = v[(i - 1)];
        }
    }
    std::cout << "The Maximum value from our STL map  library :" << maxi <<std::endl;
}
void Statistics::rmin(std::map<long, double> &mYmap) {
    std::vector<double> v;
    double mini = 0.0;
    int n = 0;
    //v[] = {0};
    for (auto it = mYmap.begin(); it != mYmap.end(); ++it)
    {
        v.push_back(it->second);
    }
    for (int i=0; i <v.size(); ++i){

        if (v[i] < v[(i - 1)]) {
            mini = v[i];
        } else {
            mini = v[(i - 1)];
        }
    }
    std::cout << "The Minimum value from our STL map  library :" << mini <<std::endl;
}
double Statistics::Onlinemean(double numbers[], double count) {
/*    double caculated_mean=0.0;
    int n=0;
    double Xn = 0.0;
    double Xnminusone = 0.0;
    double post_mean=0.0;
    double Mean=0.0;
*/
    double mean = 0;
    for (int i = 0; i < count; i++) {
	mean = mean + (numbers[i] - mean) / (i+1);}
	/* std::cout << "Number " << i << " is " << numbers[i] << std::endl;
	std::cout << i << " Mean: " << mean << std::endl;
    }*/

    return mean;


/*
	n =+ 1;
        caculated_mean+=numbers[a];
        Xn=(caculated_mean/n);
        for (int a = 1; a <count ; ++a) {
            post_mean+=numbers[a];
            Xnminusone=(post_mean/a);
            Mean= (Xnminusone+((Xn-Xnminusone)/a));
        }
    return Mean;*/
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
    std::cout<<"The Standard deviation of the elapsed time:"<<st_dev<<std::endl;
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

