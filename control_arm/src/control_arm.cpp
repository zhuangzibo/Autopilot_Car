#include"ros/ros.h"
#include"iostream"
#include"std_msgs/Float64.h"

using namespace std;

std_msgs::Float64 arm1,arm2,arm3,arm4,arm5,arm6,finger1,finger2;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"control_arm");
    ros::NodeHandle n;
    ros::Publisher chatter_pub_1=n.advertise<std_msgs::Float64>("arm/joint1_position_controller/command",10);
    ros::Publisher chatter_pub_2=n.advertise<std_msgs::Float64>("arm/joint2_position_controller/command",10);
    ros::Publisher chatter_pub_3=n.advertise<std_msgs::Float64>("arm/joint3_position_controller/command",10);
    ros::Publisher chatter_pub_4=n.advertise<std_msgs::Float64>("arm/joint4_position_controller/command",10);
    ros::Publisher chatter_pub_5=n.advertise<std_msgs::Float64>("arm/joint5_position_controller/command",10);
    ros::Publisher chatter_pub_6=n.advertise<std_msgs::Float64>("arm/joint6_position_controller/command",10);
    ros::Publisher chatter_pub_7=n.advertise<std_msgs::Float64>("arm/finger1_position_controller/command",10);
    ros::Publisher chatter_pub_8=n.advertise<std_msgs::Float64>("arm/finger2_position_controller/command",10);

    ros::Rate loop_rate(10);
    ROS_INFO("Test OK");

    while(ros::ok())
    {
        arm1.data=1.5;
        arm2.data=1.5;
        arm3.data=1.5;
        arm4.data=1.5;
        arm5.data=1.5;
        arm6.data=1.5;
        finger1.data=1.5;
        finger2.data=1.5;        
        chatter_pub_1.publish(arm1);
        chatter_pub_2.publish(arm2);
        chatter_pub_3.publish(arm3);
        chatter_pub_4.publish(arm4);
        chatter_pub_5.publish(arm5);
        chatter_pub_6.publish(arm6);
        chatter_pub_7.publish(finger1);
        chatter_pub_8.publish(finger2);
        loop_rate.sleep();
    }

    return 0;
}