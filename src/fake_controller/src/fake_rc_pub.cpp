# include "fake_rc_pub.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_rc_pub");
    ros::NodeHandle nh("~");

    FakeRcPub fake_rc_pub(nh);

    std::cout << "Fake RC Publisher is running." << std::endl;
    while(ros::ok()) {
        ros::spin();
        ros::Duration(0.01).sleep();
    }

    return 0;
}