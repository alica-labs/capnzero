//
// Created by tonmoy on 14.12.18.
//

#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Int32.h>


void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{   ros::NodeHandle n;
    ros::Publisher publisherobject=n.advertise<std_msgs::Int32>("chatter", 1);;
    ROS_INFO("I heard: [%i]", msg->data);
    std_msgs::Int32 echo_msg;
    echo_msg.data = msg->data;
    ROS_INFO("[Talker] I published [%i]\n", echo_msg.data);
    publisherobject.publish(echo_msg);

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"Parrot");
    ros::NodeHandle n;
    ros::Publisher publisherobject=n.advertise<std_msgs::Int32>("chatter", 1);
    ros::Subscriber subscriberobject=n.subscribe("echo", 1, chatterCallback) ;
    ros::Rate loop_rate(1.0);
    // enables the scape sequence ,So when we press Ctrl-C ,it returns 0 ,the loops stoped
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
