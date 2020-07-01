#include <ros/ros.h>
#include <std_msgs/String.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "robot_news_radio_transmitter"); // , ros::init_options::AnonymousName
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("/robot_news_radio", 10); // name, queue size

    ros::Rate rate(3);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "This is message from robot news radio.";
        pub.publish(msg);
        rate.sleep();
    }
}