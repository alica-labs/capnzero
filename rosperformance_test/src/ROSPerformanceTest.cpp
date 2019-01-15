//
// Created by tonmoy on 14.12.18.

#include <ros/ros.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/>
#include <sstream>
#include <vector>
#include <time.h>
#include <chrono>
#include <thread>
#include <iostream>
#include "rosperformance_test/Msgs.h"
#include "Statistics.h"

template class Statistics<double>;

long k=0;
std::map<long, std::chrono::time_point<std::chrono::high_resolution_clock >> measuringMap;
//tired to creat a map array  with rcvd msg nmbr and time took to rcv the msg
std::map<long, double >Mymap;
std::vector<double >v;
void chatterCallback(const rosperformance_test::Msgs::ConstPtr& echo_msg)
{
    long msgCount = echo_msg->number;
    auto mapEntry = measuringMap.find(msgCount);
    if (mapEntry != measuringMap.end()) {
        std::cout<<"Received ID: " << msgCount << " Time elapsed is: "
                 << std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now() - mapEntry->second).count()<< std::endl;
        //tired to store the duration inside time_passed variable
        double time_passed=double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - mapEntry->second).count() );

        // here is my zeit Map
        Mymap.emplace(msgCount,time_passed);
        //zeit.insert(std::make_pair(msgCount,time_passed));

        measuringMap.erase(msgCount);
    }
    else
    {
        std::cerr << "That is Strange!" << std::endl;
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"Talker_feedback");

    ros::NodeHandle n;
    ros::Publisher publisherobject=n.advertise<rosperformance_test::Msgs>("echo", 1);
    ros::Subscriber subscriberobject=n.subscribe("chatter", 1, chatterCallback) ;
    ros::Rate loop_rate(1.0);
    //int count = 0;



    // enables the scape sequence ,So when we press Ctrl-C ,it returns 0 ,the loops stoped
    while (ros::ok() )
    {
        std::cout << "Sending " << k << std::endl;
        rosperformance_test::Msgs msg;
        msg.number =k ;
        msg.firstdata= "Mr.Ros";
        msg.seconddata="A vector is a dynamically sized sequence of objects that provides array-style operator[] random access. The member function push_back copies its argument via copy constructor, adds that copy as the last item in the vector, and increments its size by one. pop_back does the exact opposite, by removing the last element. Inserting or deleting items from the end of a vector takes amortized constant time, and inserting or deleting from any other location takes linear time. These are the basics of vectors. There is a lot more to them.\n"
                       "\n"
                       "In most cases, a vector should be your first choice over a C-style array. First of all, they are dynamically sized, which means they can grow as needed. You don’t have to do all sorts of research to figure out an optimal static size, as in the case of C arrays; a vector grows as needed, and it can be resized larger or smaller manually if you need to. Second, vectors offer bounds checking with the at member function (but not with operator[]), so that you can do something if you reference a nonexistent index instead of simply watching your program crash or worse, continuing execution with corrupt data. Look at Example 4-7. It shows how to deal with out-of-bounds indexes";
        //msg.data = count;

        measuringMap.emplace(k, std::chrono::high_resolution_clock::now());

        publisherobject.publish(msg);
        //ros::spinOnce();

        ros::AsyncSpinner spinner(4);
        spinner.start();
        loop_rate.sleep();
        k ++;
    }

    sleep(2);


    std::cout << "We must have missed " << measuringMap.size() << " Number of Msgs!" <<std::endl;
    for (auto& entry : measuringMap) {
        std::cout << "ID: " << entry.first << " StartTime: " << std::chrono::duration_cast<std::chrono::milliseconds>(entry.second.time_since_epoch()).count() << std::endl;
    }
    // taking all data into array
    std::map<long, double>::iterator p;
    for (p= Mymap.begin(); p != Mymap.end(); ++p){
        v.push_back(p->second);
    }
    // Vectror to array conversion
    double *arr= v.data();
    double datasize=(v.size()) ;
    std::cout << "number of rcvd msg: "<<datasize<<std::endl ;

    Statistics<double> st;

    st.referencemean(Mymap);
    st.referencestd_dev(Mymap);
    st.rmax(Mymap);
    st.rmin(Mymap);




    double OnlineMeani=st.Onlinemean(arr,datasize);
    st.Onlinemeanprinter(OnlineMeani);
    double Traditionalmean=st.mean(arr,datasize) ;
    st.meanprinter(Traditionalmean);
    double Traditionalstd_dev=st.standard_dev(arr,datasize);
    st.stdvprinter(Traditionalstd_dev);
    auto maximum=st.max(arr,datasize);
    st.maxiprinter(maximum);
    auto minimum=st.mini(arr,datasize);
    st.miniprinter(minimum);

    std::cout << "Cleaning up now."  << std::endl;
    //delete arr;
    v.clear();
    Mymap.clear();
    measuringMap.clear();
    
    return 0;

}






