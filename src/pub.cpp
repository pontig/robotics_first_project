#include <sstream>
#include <tf/transform_broadcaster.h>

#include "first_project/Odom.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "service/ResetOdom.h"
#include "std_msgs/String.h"

#define D 2.8

class pub_sub {
    geometry_msgs::Quaternion message;

   public:
    pub_sub() {
        ssteer_sub = n.subscribe("/speed_steer", 1000, &pub_sub::callback, this);
        service = n.advertiseService("reset_odom", &pub_sub::reset, this);

        custom_pub = n.advertise<first_project::Odom>("custom_odometry", 1);
        regular_pub = n.advertise<nav_msgs::Odometry>("odometry", 1);

        n.getParam("/starting_x", custom_odom.x);
        n.getParam("/starting_y", custom_odom.y);
        n.getParam("/starting_th", custom_odom.theta);

        time = ros::Time::now();
    }

    void callback(const geometry_msgs::Quaternion::ConstPtr& msg) {
        message = *msg;
        double speed = message.x;
        double steering_angle = message.y;

        custom_messages::Odom custom_odom_msg = compute_custom_odom(message);
        custom_pub.publish(custom_odom_msg);

        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = custom_odom_msg.x;
        odom_msg.pose.pose.position.y = custom_odom_msg.y;
        odom_msg.pose.pose.position.z = 0;
        odom_msg.pose.pose.orientation.z = custom_odom_msg.theta;

        odom_msg.twist.twist.linear.x = speed * cos(custom_odom_msg.theta);
        odom_msg.twist.twist.linear.y = speed * sin(custom_odom_msg.theta);
        odom_msg.twist.twist.angular.z = speed * tan(steering_angle) / D;

        regular_pub.publish(odom_msg);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(custom_odom_msg.x, custom_odom_msg.y, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, custom_odom_msg.theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    }

    bool reset(service::ResetOdom::Request& req, service::ResetOdom::Response& res) {
        
        time = ros::Time::now();
        
        custom_odom.x ;
        custom_odom.y ;
        custom_odom.theta ;
        custom_odom.newTime = time;

        ROS_INFO("sending back response: [%d]", (bool)res.reset);
        res.resetted = true;

        return true;
    }

   private:
    ros::NodeHandle n;

    ros::Subscriber ssteer_sub;
    ros::ServiceServer service;
    tf::TransformBroadcaster br;
    custom_messages::Odom custom_odom;
    custom_odom.x = 0;
    custom_odom.y = 0;
    custom_odom.theta = 0;
    ros::Time time;

    ros::Publisher custom_pub;
    ros::Publisher regular_pub;

    custom_messages::Odom compute_custom_odom(geometry_msgs::Quaternion msg) {
        double speed = msg.x;
        double steer = msg.y;
        double newTime = ros::Time::now();
        double t = (newTime - time).toSec();

        custom_messages::Odom odom_msg;
        double r = D / (tan(steer));
        double omega = speed / r;

        odom_msg.x = custom_odom.x + speed * t * cos(custom_odom.theta + omega * t / 2);
        odom_msg.y = custom_odom.y + speed * t * sin(custom_odom.theta + omega * t / 2);
        odom_msg.theta = custom_odom.theta + omega * t;
        odom_msg.timestamp = newTime;
        time = newTime;

        custom_odom = odom_msg;
        return odom_msg;
    }

}

int
main(int argc, char const* argv[]) {
    ros::init(argc, argv, "odom_node");
    pub_sub my_pub_sub;
    ros::spin();

    return 0;
}
