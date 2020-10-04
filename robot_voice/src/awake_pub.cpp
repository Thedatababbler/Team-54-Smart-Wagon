#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "std_msgs/String.h"
#include <ros/ros.h>

int flag = 1;

//void ReadyUp(const std_msgs::String::ConstPtr& msg

int main(int argc, char* argv[])
    {
        ros::init(argc, argv, "voiceAwake");
        ros::NodeHandle n;
        ros::Rate loop_rate(1); //loop waiting for input 
	    //ros::Subscriber wakeUpSub = n.subscribe("voiceWakeupready", 1000, ReadyUp); 
        ros::Publisher voiceWakeup = n.advertise<std_msgs::String>("voiceWakeup", 3);
        while(ros::ok()){
            if (flag){
                std_msgs::String msg;
	            msg.data = "wake up!";
	            voiceWakeup.publish(msg);
                //flag = 0;
            }
            loop_rate.sleep();
        }
        return 0;
    }
