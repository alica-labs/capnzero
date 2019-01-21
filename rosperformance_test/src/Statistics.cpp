//
// Created by tonmoy on 02.01.19.
//
#include "Statistics.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

template <typename x>
void Statistics<x>::referencemean(std::map<long, double> &mYmap) {
    x mean = 0;
    int n=0;
    for (auto it = mYmap.begin();
         it != mYmap.end(); ++it){
        n += 1;
        mean = mean + (((it->second) - mean) /n); // As I am using my dividend in double format and divisor in int ,the quotient will be double .
    }                                             // source-https://stackoverflow.com/questions/7571326/why-does-dividing-two-int-not-yield-the-right-value-when-assigned-to-double
std::cout<< "Mean from our STL map library: "<<mean<<" ms"<<std::endl;
}
template <typename x>
void Statistics<x>::referencestd_dev(std::map<long, double> &mYmap) {
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
    std::cout<< "Std_dev from our STL map library: "<<st_dev<<" ms"<<std::endl;
}

template <typename x>
void Statistics<x>::rmax(std::map<long, double> &mYmap) {
    std::vector<x> v;

    for (auto it = mYmap.begin(); it != mYmap.end(); ++it) {
        v.push_back(it->second);

    }
    auto maxV=*max_element(v.begin(),v.end());
    std::cout << "The maximum value from our STL map  library: " << maxV <<" ms"<<std::endl;
}
template <typename x>
void Statistics<x>::rmin(std::map<long, double> &mYmap) {
    std::vector<x> v;
    for (auto it = mYmap.begin(); it != mYmap.end(); ++it)
    {
        v.push_back(it->second);
    }
    auto minV=*min_element(v.begin(),v.end());
    std::cout << "The Minimum value from our STL map  library: " << minV <<" ms"<<std::endl;
}

template class Statistics<double>;

/*
template <typename x>
x  Statistics<x>::Onlinemean(x  numbers[], x  count) {
    double caculated_mean=0.0;
    int n=0;
    double Xn = 0.0;
    double Xnminusone = 0.0;
    double post_mean=0.0;
    double Mean=0.0;

    x  mean = 0;
    for (int i = 0; i < count; i++) {
	mean = mean + (numbers[i] - mean) / (i+1);}
	 std::cout << "Number " << i << " is " << numbers[i] << std::endl;
	std::cout << i << " Mean: " << mean << std::endl;
    }

    return mean;



	n =+ 1;
        caculated_mean+=numbers[a];
        Xn=(caculated_mean/n);
        for (int a = 1; a <count ; ++a) {
            post_mean+=numbers[a];
            Xnminusone=(post_mean/a);
            Mean= (Xnminusone+((Xn-Xnminusone)/a));
        }
    return Mean;
}
template <typename x>
x  Statistics<x>::mean(x  numbers[], x  count) {
    x  caculated_mean=0.0;

    x  post_mean=0.0;
    for (int a=0;a < count; a++)
    {
        caculated_mean+=numbers[a];
    }
    caculated_mean/= x (count);
    return caculated_mean;
}
template <typename x>
x  Statistics<x>::standard_dev(x  numbers[], x  count) {
    x  st_dev=0.0;
    x  average= mean(numbers,count);
    x  temp_dev;
    for (int a=0;a < count; a++)
    {
        temp_dev=numbers[a]-average;
        st_dev +=temp_dev*temp_dev;
    }
    st_dev /=(count);
    st_dev = std::sqrt(st_dev);
    return st_dev;
}
template <typename x>
void Statistics<x>::meanprinter(x  mEan) {
    std::cout<<"The mean of the elapsed time: "<<mEan<<std::endl;
}
template <typename x>
void Statistics<x>::stdvprinter(x  sTddv) {
    std::cout<<"The Standard deviation of the elapsed time:"<<sTddv<<std::endl;
}
template <typename x>
void Statistics<x>::maxiprinter(x  mAx) {
    std::cout<<"The max of the elapsed time:"<<mAx<<std::endl;
}
template <typename x>
void Statistics<x>::miniprinter(x  mIn) {
    std::cout<<"The min of the elapsed time:"<<mIn<<std::endl;

}
template <typename x>
void Statistics<x>::Onlinemeanprinter(x  meAn) {
    std::cout<<"The Online mean of the elapsed time:"<<meAn<<std::endl;
}
template <typename x>
x  Statistics<x>::max(x  numbers[], int count) {
    x  maxi=numbers[0];

    for (int j = 1; j < (count); ++j)
    {
        if(numbers[j]>maxi)
        {
            maxi=numbers[j];
        }
        else
        {
            //maxi=numbers[j-1];
        }

    }

    return maxi;
}
template <typename x>
x  Statistics<x>::mini(x  numbers[], int count) {
    x  mini=numbers[0];
    for (int k = 1; k < (count); ++k) {

        if(mini>numbers[k])
        {
            mini=numbers[k];
        }
    }
    return mini;
}*/

//Explicitly instantiate the template, and its member definitions
// in this approach, you should ensure that all the of the implementation is placed into one .cpp file (i.e. one translation unit) and that the explicit instantation is placed after the definition of all the functions (i.e. at the end of the file
// more info --https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
