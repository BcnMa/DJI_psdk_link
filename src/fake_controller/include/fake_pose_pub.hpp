# include <iostream>

# include "ros/ros.h"
# include "sensor_msgs/NavSatFix.h"


class FakePosePub {
private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Timer timer_pub;


public:
    FakePosePub(ros::NodeHandle &main_nh) {
        nh = main_nh;
        pose_pub = nh.advertise<sensor_msgs::NavSatFix>("pose_ctrl", 10);
        timer_pub = nh.createTimer(ros::Duration(0.1), &FakePosePub::pub_control, this);
    }

    void pub_control(const ros::TimerEvent& event) {
        sensor_msgs::NavSatFix pose_data;

        pose_data.header.stamp = ros::Time::now();
        pose_data.header.frame_id = "map";

        pose_data.latitude = 30.0;
        pose_data.longitude = 120.0;
        pose_data.altitude = 10.0;

        pose_pub.publish(pose_data);
    }


}; // class FakeRcPub