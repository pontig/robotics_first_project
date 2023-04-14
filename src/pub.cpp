#include <sstream>

#include "first_project/Odom.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#define D 2.8

class pub_sub {
    geometry_msgs::Quaternion message;

   public:
    pub_sub() {
        ssteer_sub = n.subscribe("/speed_steer", 1000, &pub_sub::SScallback, this);

        custom_pub = n.advertise<first_project::Odom>("custom_odometry", 1);
        regular_pub = n.advertise<nav_msgs::Odometry>("odometry", 1);
    }

    void callback(const geometry_msgs::Quaternion::ConstPtr& msg) {
        message = *msg;

        custom_messages::Odom custom_odom_msg = compute_custom_odom(message);
        custom_pub.publish(custom_odom_msg);

        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = custom_odom_msg.x;
        odom_msg.pose.pose.position.y = custom_odom_msg.y;
        odom_msg.pose.pose.orientation.z = custom_odom_msg.theta;

        odom_msg.twist.twist.linear.x = message.x * cos(custom_odom_msg.theta);
        odom_msg.twist.twist.linear.y = message.x * sin(custom_odom_msg.theta);
        odom_msg.twist.twist.angular.z = message.x / D * tan(message.y);


    }

   private:
    ros::NodeHandle n;

    ros::Subscriber ssteer_sub;
    custom_messages Odom custom_odom;
    custom_odom.x = 0;
    custom_odom.y = 0;
    custom_odom.theta = 0;
    // TODO: add timestamp

    ros::Publisher custom_pub;
    ros::Publisher regular_pub;

    custom_messages::Odom compute_custom_odom(geometry_msgs::Quaternion msg) {
        double speed = msg.x;
        double steer = msg.y;

        custom_messages::Odom odom_msg;
        double r = D / (tan(steer));
        double omega = speed / r;

        odom_msg.x = custom_odom.x + speed * t * cos(custom_odom.theta + omega * t / 2);
        odom_msg.y = custom_odom.y + speed * t * sin(custom_odom.theta + omega * t / 2);
        odom_msg.theta = custom_odom.theta + omega * t;
        // TODO: add timestamp

        custom_odom = odom_msg;
        return odom_msg;
    }

    nav_msgs::Odometry compute_odom(geometry_msgs::Quaternion msg) {
        double speed = msg.x;
        double steer = msg.y;

        nav_msgs::Odometry odom_msg;
        double r = D / (tan(steer));
        double omega = speed / r;

        odom_msg.pose.pose.position.x = custom_odom.x + speed * t * cos(custom_odom.theta + omega * t / 2);
        odom_msg.pose.pose.position.y = custom_odom.y + speed * t * sin(custom_odom.theta + omega * t / 2);
        odom_msg.pose.pose.orientation.z = custom_odom.theta + omega * t;
}

int
main(int argc, char const* argv[]) {
    ros::init(argc, argv, "odom_node");
    pub_sub my_pub_sub;
    ros::spin();

    return 0;
}

