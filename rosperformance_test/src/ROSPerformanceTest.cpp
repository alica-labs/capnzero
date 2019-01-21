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
// template class Statistics<double>;
long k=0;
std::map<long, std::chrono::time_point<std::chrono::high_resolution_clock >> measuringMap;
//tired to creat a map array  with rcvd msg nmbr and time took to rcv the msg
std::map<long, double >Mymap; //container with data type long and double
std::vector<double >v;
std::pair<long ,std::chrono::time_point<std::chrono::high_resolution_clock >>p1;
void chatterCallback(const rosperformance_test::Msgs::ConstPtr& echo_msg)
{
    long msgCount = echo_msg->number;
    auto mapEntry = measuringMap.find(msgCount);
    if (mapEntry != measuringMap.end()) {
        std::cout<<"Received ID: " << msgCount << " Time elapsed is: "
                 << std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now() - mapEntry->second).count()<<" ms"<<std::endl;
        //tired to store the duration inside time_passed variable
        double time_passed=double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - mapEntry->second).count() );

        // here is my Mymap is the Map containner
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
    // enables the scape sequence ,So when we press Ctrl-C ,it returns 0 ,the loops stoped
    while (ros::ok() )
    {
        std::cout << "Sending " << k << std::endl;
        rosperformance_test::Msgs msg;
        msg.number =k ;
        msg.firstdata= "Mr.Ros";
        char s1[1100]="Apart from counting words and characters, our online editor can help you to improve word choice and writing style, and, optionally, help you to detect grammar mistakes and plagiarism. To check word count, simply place your cursor into the text box above and start typing. You'll see the number of characters and words increase or decrease as you type, delete, and edit them. You can also copy and paste text from another program over into the online editor above. The Auto-Save feature will make sure you won't lose any changes while editing, even if you leave the site and come back later. Tip: Bookmark this page now."
                      "+" "Knowing the word count of a text can be important. For example, if an author has to write a minimum or maximum amount of words for an article, essay, report, story, book, paper, you name it. WordCounter will help to make sure its word count reaches a specific requirement or stays within a certain limit."
                      "+" "In the Details overview you can see the average speaking and reading time for y.This can perform really well depending on the usages";
        msg.seconddata =s1;

        measuringMap.emplace(k, std::chrono::high_resolution_clock::now());
        p1 = std::make_pair(k,std::chrono::high_resolution_clock::now());

        publisherobject.publish(msg);
        //ros::spinOnce();
        std::cout<<"The published signal: "<<p1.first<<"   Time took to publish: "<<std::chrono::duration_cast<std::chrono::seconds>(p1.second.time_since_epoch()).count() <<" ms "<<std::endl;
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
    /*
    // Vectror to array conversion
    double *arr= v.data();
    double datasize=(v.size()) ;*/


    std::cout << "number of rcvd msg: "<<Mymap.size()<<std::endl ;

    Statistics<double> st;

    st.referencemean(Mymap);
    st.referencestd_dev(Mymap);
    st.rmax(Mymap);
    st.rmin(Mymap);




    /*double OnlineMeani=st.Onlinemean(arr,datasize);
    st.Onlinemeanprinter(OnlineMeani);
    double Traditionalmean=st.mean(arr,datasize) ;
    st.meanprinter(Traditionalmean);
    double Traditionalstd_dev=st.standard_dev(arr,datasize);
    st.stdvprinter(Traditionalstd_dev);
    auto maximum=st.max(arr,datasize);
    st.maxiprinter(maximum);
    auto minimum=st.mini(arr,datasize);
    st.miniprinter(minimum); */

    std::cout << "Cleaning up now. "  << std::endl;
    //delete arr;
    //v.clear();
    Mymap.clear();
    measuringMap.clear();
    return 0;

}






