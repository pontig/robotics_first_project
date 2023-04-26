#include <sstream>
#include <tf/transform_broadcaster.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "first_project/Odom.h"
#include "first_project/ResetOdom.h"

/*
    \brief Distance between front and back wheels
*/
#define D 2.8

class pub_sub {

    public:
        pub_sub() {
            // Subscriber 
            ssteer_sub  = n.subscribe("/speed_steer", 1000, &pub_sub::callback, this);
            // Service
            service     = n.advertiseService("reset_odom", &pub_sub::reset, this);
            // Publishers 
            custom_pub  = n.advertise<first_project::Odom>("custom_odometry", 1);
            regular_pub = n.advertise<nav_msgs::Odometry>("odometry", 1);

            n.getParam("/starting_x",  custom_odom.x);
            n.getParam("/starting_y",  custom_odom.y);
            n.getParam("/starting_th", custom_odom.th);

            time = ros::Time::now();
        }

        void callback(const geometry_msgs::Quaternion::ConstPtr& msg) {
            message = *msg;

            // Compute the custom Odometry message (x, y, theta, timestamp)
            // from the Quaternion message 
            // Publish it
            first_project::Odom custom_odom_msg = compute_custom_odom(message);
            custom_pub.publish(custom_odom_msg);

            // Compute the standard Odometry message (Pose and Twist)
            // from the Quaternion message and the custom Odometry message
            // Publish it
            nav_msgs::Odometry odom_msg = compute_odom(message, custom_odom_msg);
            regular_pub.publish(odom_msg);

            // TF 
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(custom_odom_msg.x, custom_odom_msg.y, 0));
            tf::Quaternion q;
            q.setRPY(0, 0, custom_odom_msg.th);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

        }

        bool reset(first_project::ResetOdom::Request& req, first_project::ResetOdom::Response& res) {
            time = ros::Time::now();
            
            // reset the custom Odometry message parameters 
            custom_odom.x ; //TODO use n.getParam instead ?? 
            // No, the assignment says that reset should set the parameters to 0, not to starting value
            custom_odom.y ;
            custom_odom.th;
            custom_odom.timestamp = std::to_string(time.toSec());

            ROS_INFO("sending back response: [%d]", (bool)res.resetted);
            res.resetted = true;

            return true;
        }

    private:
        ros::NodeHandle n;

        ros::Subscriber  ssteer_sub;
        ros::ServiceServer  service;
        ros::Publisher   custom_pub;
        ros::Publisher  regular_pub;

        tf::TransformBroadcaster br;

        // Custom Odometry message (x, y, theta, timestamp)
        first_project::Odom custom_odom;
        // Quaternion message received 
        geometry_msgs::Quaternion message;   // TODO here ? ptr? ConstPtr&

        ros::Time time;

        first_project::Odom compute_custom_odom(geometry_msgs::Quaternion msg) {
            double speed = msg.x;
            double steering_angle = msg.y;

            // Ackerman Steering (Bicycle Approximation):
            // r = radius from ICC to center of the vehicle 
            double r = D / (tan(steering_angle));
            // omega = angular velocity  
            double omega = speed / r;

            // Compute delta t 
            ros::Time new_time = ros::Time::now();
            double dt = (new_time - time).toSec();

            // new custom Odometry message 
            first_project::Odom new_custom_odom;
            // custom_odom     = custom Odometry message at time
            // new_custom_odom = custom Odometry message at new_time
            new_custom_odom.x  = custom_odom.x + speed * dt * cos(custom_odom.th + omega * dt / 2);
            new_custom_odom.y  = custom_odom.y + speed * dt * sin(custom_odom.th + omega * dt / 2);
            new_custom_odom.th = custom_odom.th + omega * dt;
            new_custom_odom.timestamp = std::to_string(new_time.toSec());
            time = new_time;

            custom_odom = new_custom_odom;
            return new_custom_odom;
        }

        nav_msgs::Odometry compute_odom(geometry_msgs::Quaternion msg, 
                                        first_project::Odom custom_odom_msg) {
            double speed = msg.x;
            double steering_angle = msg.y;

            // new Odometry message
            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = custom_odom_msg.x;
            odom.pose.pose.position.y = custom_odom_msg.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation.z = custom_odom_msg.th;

            odom.twist.twist.linear.x  = speed * cos(custom_odom_msg.th);
            odom.twist.twist.linear.y  = speed * sin(custom_odom_msg.th);
            odom.twist.twist.angular.z = speed * tan(steering_angle) / D;

            return odom; 
        }

}; 

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_node");
    pub_sub my_pub_sub;
    ros::spin();

    return 0;
}
