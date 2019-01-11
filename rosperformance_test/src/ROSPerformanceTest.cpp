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

    Statistics st;

    // enables the scape sequence ,So when we press Ctrl-C ,it returns 0 ,the loops stoped
    while (ros::ok() )
    {
        std::cout << "Sending " << k << std::endl;
        rosperformance_test::Msgs msg;
        msg.number =k ;
        msg.firstdata= "Mr.Ros";
        msg.seconddata="Was ist ROS?\n"
                     "\n"
                     "ROS ist ein Open Source, Meta-Betriebssystem für deinen Roboter. Es stellt Dienste zur Verfügung, welche du von einem Betriebssystem erwartest: Hardwareabstraktion, Gerätetreiber, Utilityfunktionen, Interprozesskommunikation und Paketmanagment. Des Weiteren sind Werkzeuge und Bibliotheken für das Beziehen, Builden, Schreiben und Ausführen von Code über mehrere Computer vorhanden. ROS kann in einigen Aspekten mit anderen Roboterframeworks verglichen werden. Dazu gehören: Player, YARP, Orocos, CARMEN, Orca, MOOS sowie Microsoft Robotics Studio.\n"
                     "\n"
                     "Ziele\n"
                     "\n"
                     "Viele fragen sich worin sich ROS von anderen Roboter Frameworks unterscheidet. Diese Frage ist schwer zu beantworten, da ROS nicht zum Ziel hat ein Framework mit den meisten Funktionen zu sein. Stattdessen ist der Hauptzweck von ROS die Wiederverwendung von Code in der Roboterforschung und -entwicklung. ROS ist ein verteiltes System von Prozessen (Nodes), welches die lose Kopplung von individuellen Komponenten ermöglic ";
        //std_msgs::Int32 msg;
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
    double Traditionalmean=st.mean(arr,datasize) ;
    st.meanprinter(Traditionalmean);
    double OnlineMeani=st.Onlinemean(arr,datasize);
    st.Onlinemeanprinter(OnlineMeani);
    st.refrencemean(Mymap);
    st.refrencestd_dev(Mymap);

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






