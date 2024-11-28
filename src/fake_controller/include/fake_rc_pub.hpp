# include <iostream>
# include <deque>

# include "ros/ros.h"
# include "geometry_msgs/Twist.h"
# include "nav_msgs/Odometry.h"
# include "nav_msgs/Path.h"
# include "tf/transform_datatypes.h" 

class FakeRcPub {
private:
    ros::NodeHandle nh;
    ros::Publisher rc_pub, odom_pub, path_pub;
    ros::Timer timer_pub;

    int current_pub_num = 0;
    int max_pub_num = 100;

    double normal_speed = 5.0;
    double x = 0.0, y = 0.0, theta = 0.0;

    std::deque<geometry_msgs::PoseStamped> path_points;
    nav_msgs::Path path_msg;

public:
    FakeRcPub(ros::NodeHandle &main_nh) {
        nh = main_nh;
        rc_pub = nh.advertise<geometry_msgs::Twist>("/rc_ctrl", 10);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/fake_odom", 10);
        path_pub = nh.advertise<nav_msgs::Path>("/fake_path", 10);
        timer_pub = nh.createTimer(ros::Duration(0.2), &FakeRcPub::pub_control, this);
        path_msg.header.frame_id = "odom";
    }

    void pub_control(const ros::TimerEvent& event) {
        // 1. speed
        geometry_msgs::Twist cmd_data;

        double cos_theta = sin(2 * M_PI * current_pub_num / max_pub_num);
        double sin_theta = - cos(2 * M_PI * current_pub_num / max_pub_num);

        cmd_data.linear.x = normal_speed * cos_theta;
        cmd_data.linear.y = normal_speed * sin_theta;
        cmd_data.linear.z = 0;
        
        cmd_data.angular.x = 0;
        cmd_data.angular.y = 0;
        cmd_data.angular.z = 0;

        // 2. odom
        double dt = 0.1; 
        double delta_x = (cmd_data.linear.x * cos(theta) - cmd_data.linear.y * sin(theta)) * dt;
        double delta_y = (cmd_data.linear.x * sin(theta) + cmd_data.linear.y * cos(theta)) * dt;
        double delta_theta = cmd_data.angular.z * dt;          

        x += delta_x;
        y -= delta_y;
        theta += delta_theta;

        if (theta > 2 * M_PI) theta -= 2 * M_PI;
        if (theta < 0) theta += 2 * M_PI;

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;


        tf::Quaternion q = tf::createQuaternionFromYaw(theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = cmd_data.linear.x;
        odom_msg.twist.twist.angular.z = cmd_data.angular.z;

        // 3. path
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        if (path_points.size() >= 30) {
            path_points.pop_front();
        }
        path_points.push_back(pose_stamped);
        path_msg.poses.clear();  
        for (const auto& point : path_points) {
            path_msg.poses.push_back(point);  
        }

        rc_pub.publish(cmd_data);
        odom_pub.publish(odom_msg);
        path_pub.publish(path_msg);

        current_pub_num++;
    }


}; // class FakeRcPub