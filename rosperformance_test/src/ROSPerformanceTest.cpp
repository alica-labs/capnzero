//
// Created by tonmoy on 14.12.18.

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <time.h>
#include <chrono>
#include <thread>
#include <iostream>

std::map<int, std::chrono::time_point<std::chrono::steady_clock>> measuringMap;

struct Timer
        {
    std::chrono::time_point<std::chrono::steady_clock> start,end;
    std::chrono::duration<float> duration;

    Timer()
    {
        start = std::chrono::steady_clock::now() ;
    }
    ~Timer()
    {
        end = std::chrono::steady_clock::now();
        duration = end - start;
        float ms=duration.count() *1000.0f;
        std::cout<<"Timer took"<<ms<<"ms"<<std::endl;
    }
};
void chatterCallback(const std_msgs::Int32::ConstPtr& echo_msg)
{
    int msgCount = echo_msg->data;
    auto mapEntry = measuringMap.find(msgCount);
    if (mapEntry != measuringMap.end()) {
        std::cout<<"Received ID: " << msgCount << " Time elapsed is: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - mapEntry->second).count() << std::endl;
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

    // enables the scape sequence ,So when we press Ctrl-C ,it returns 0 ,the loops stoped
    while (ros::ok() && count < 1000)
    {
        std::cout << "Sending " << count << std::endl;

        std_msgs::Int32 msg;
        msg.data = count;

        measuringMap.emplace(count, std::chrono::steady_clock::now());

        publisherobject.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    sleep(2);

    std::cout << "We must have missed " << measuringMap.size() << " Number of Msgs!" <<std::endl;
    for (auto& entry : measuringMap) {
        std::cout << "ID: " << entry.first << " StartTime: " << std::chrono::duration_cast<std::chrono::milliseconds>(entry.second.time_since_epoch()).count() << std::endl;
    }

    return 0;

}






