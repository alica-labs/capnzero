//
// Created by tonmoy on 14.12.18.

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <time.h>
#include <chrono>
#include <thread>
#include <iostream>

#include "Statistics.cpp"

std::map<int, std::chrono::time_point<std::chrono::high_resolution_clock >> measuringMap;
//tired to creat a map array  with rcvd msg nmbr and time took to rcv the msg
std::map<int, double >zeit;
std::vector<double >v;
void chatterCallback(const std_msgs::Int32::ConstPtr& echo_msg)
{
    int msgCount = echo_msg->data;
    auto mapEntry = measuringMap.find(msgCount);
    if (mapEntry != measuringMap.end()) {
        std::cout<<"Received ID: " << msgCount << " Time elapsed is: "
        << std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now() - mapEntry->second).count()<< std::endl;
        //tired to store the duration inside time_passed variable
        double time_passed=double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - mapEntry->second).count() );

        // here is my zeit Map
        zeit.emplace(msgCount,time_passed);
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
    ros::Publisher publisherobject=n.advertise<std_msgs::Int32>("echo", 1);
    ros::Subscriber subscriberobject=n.subscribe("chatter", 1, chatterCallback) ;
    ros::Rate loop_rate(1.0);
    int count = 0;
    Statistics st;

    // enables the scape sequence ,So when we press Ctrl-C ,it returns 0 ,the loops stoped
    while (ros::ok() && count < 1000)
    {
        std::cout << "Sending " << count << std::endl;

        std_msgs::Int32 msg;
        msg.data = count;

        measuringMap.emplace(count, std::chrono::high_resolution_clock::now());

        publisherobject.publish(msg);
        //ros::spinOnce();

        ros::AsyncSpinner spinner(4);
        spinner.start();
        loop_rate.sleep();
        count++;
    }

    sleep(2);


    std::cout << "We must have missed " << measuringMap.size() << " Number of Msgs!" <<std::endl;
    for (auto& entry : measuringMap) {
        std::cout << "ID: " << entry.first << " StartTime: " << std::chrono::duration_cast<std::chrono::milliseconds>(entry.second.time_since_epoch()).count() << std::endl;
    }
    // taking alla data into array
    std::map<int, double>::iterator p;
    for (p= zeit.begin(); p != zeit.end(); ++p){
        v.push_back(p->second);
    }
    // Vectror to array conversion
    double *arr= v.data();
    double mumu=(v.size()) ;
    std::cout << "number of rcvd msg: "<<mumu<<std::endl ;
    double kuku=st.mean(arr,mumu) ;
    st.meanprinter(kuku);
    double muku=st.standard_dev(arr,mumu);
    st.stdvprinter(muku);
    auto maximum=st.max(arr,mumu);
    st.maxiprinter(maximum);
    auto minimum=st.mini(arr,mumu);
    st.miniprinter(minimum);
    v.clear();
    zeit.clear();
    delete arr;
    measuringMap.clear();
    return 0;

}






