# include "fake_rc_pub.hpp"
# include "psdk_msgs/DroneState.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_rc_pub");
    ros::NodeHandle nh("~");
    ros::Publisher drone_state_pub = nh.advertise<psdk_msgs::DroneState>("/drone_state", 100000);
    ros::Rate loop_rate(100);

    FakeRcPub fake_rc_pub(nh);
    psdk_msgs::DroneState drone_state_msg;

    drone_state_msg.state = 5;
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < ros::Duration(2.0)) {
        drone_state_pub.publish(drone_state_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Fake RC Publisher is running." << std::endl;
    while(ros::ok()) {
        ros::spin();
        ros::Duration(0.01).sleep();
    }

    return 0;
}