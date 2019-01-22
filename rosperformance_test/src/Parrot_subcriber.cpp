//
// Created by tonmoy on 14.12.18.
//

#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Int32.h>
#include "rosperformance_test/Msgs.h"
#include <sstream>


void chatterCallback(const rosperformance_test::Msgs& msg)
{   ros::NodeHandle n;
    ros::Publisher publisherobject=n.advertise<rosperformance_test::Msgs>("chatter", 1);;
    ROS_INFO("I heard: [%i]", msg.number);
    rosperformance_test::Msgs echo_msg;
    echo_msg = msg;
    ROS_INFO("[Talker] I published [%i]\n", echo_msg.number);
    publisherobject.publish(echo_msg);

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"Parrot");
    ros::NodeHandle n;
    ros::Publisher publisherobject=n.advertise<rosperformance_test::Msgs>("chatter", 1);
    ros::Subscriber subscriberobject=n.subscribe("echo", 1, chatterCallback) ;
    ros::Rate loop_rate(1.0);
    // enables the scape sequence ,So when we press Ctrl-C ,it returns 0 ,the loops stoped
    while (ros::ok())
    {
        // ros::spin();
        ros::AsyncSpinner spinner(4);
        spinner.start();
        loop_rate.sleep();
    }
    return 0;

}

