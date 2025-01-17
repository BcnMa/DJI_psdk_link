# include "iostream"
# include "deque"
# include "thread"
# include "mutex"
# include "unistd.h"

# include "ros/ros.h"
# include "geometry_msgs/Twist.h"
# include "nav_msgs/Odometry.h"
# include "nav_msgs/Path.h"
# include "tf/transform_datatypes.h" 

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

class FakeController {
private:
    ros::Publisher drone_rc_pub, drone_odom_pub, drone_path_pub;
    ros::Publisher vehicle_cmd_vel_pub, vehicle_odom_pub, vehicle_path_pub;
    ros::Timer timer_drone_pub, timer_vehicle_pub;

    std::mutex drone_mtx, vehicle_mtx;
    geometry_msgs::Twist drone_cmd_data, vehicle_cmd_data;

    int current_drone_pub_num = 0, current_vehicle_pub_num = 0;;
    int max_pub_num = 500;

    double normal_speed = 3.0;
    double drone_x = 0.0, drone_y = 0.0, drone_theta = 0.0;
    double vehicle_x = 0.0, vehicle_y = 0.0, vehicle_theta = 0.0;

    std::deque<geometry_msgs::PoseStamped> drone_path_points, vehicle_path_points;
    nav_msgs::Path drone_path_msg, vehicle_path_msg;

public:
    FakeController();
    ~FakeController() = default;

    void pub_drone_control(const ros::TimerEvent& event);
    void pub_vehicle_control(const ros::TimerEvent& event);

    void drone_cmd_reset();
    void vehicle_cmd_reset();

    void drone_cmd_line(const double &speed);
    void drone_cmd_circle(const double &line_speed, const double &angular_speed);

    void vehicle_cmd_line(const double &speed);
    void vehicle_cmd_circle(const double &line_speed, const double &angular_speed);

}; // class FakeRcPub


FakeController::FakeController() {
    ros::NodeHandle nh;
    vehicle_cmd_reset();
    drone_cmd_reset();

    drone_rc_pub = nh.advertise<geometry_msgs::Twist>("/rc_ctrl", 10);
    drone_odom_pub = nh.advertise<nav_msgs::Odometry>("/fake_drone_odom", 10);
    drone_path_pub = nh.advertise<nav_msgs::Path>("/fake_drone_path", 10);

    vehicle_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vehicle_odom_pub = nh.advertise<nav_msgs::Odometry>("/fake_vehicle_odom", 10);
    vehicle_path_pub = nh.advertise<nav_msgs::Path>("/fake_vehicle_path", 10);

    timer_drone_pub = nh.createTimer(ros::Duration(0.02), &FakeController::pub_drone_control, this);
    timer_vehicle_pub = nh.createTimer(ros::Duration(0.1), &FakeController::pub_vehicle_control, this);
    drone_path_msg.header.frame_id = "odom";
    vehicle_path_msg.header.frame_id = "odom";
}


void FakeController::drone_cmd_reset() {
    geometry_msgs::Twist cmd_data;

    cmd_data.linear.x = 0;
    cmd_data.linear.y = 0;
    cmd_data.linear.z = 0;
    
    cmd_data.angular.x = 0;
    cmd_data.angular.y = 0;
    cmd_data.angular.z = 0;

    std::lock_guard<std::mutex> lock(drone_mtx);
    drone_cmd_data = cmd_data;
}

void FakeController::vehicle_cmd_reset() {
    geometry_msgs::Twist cmd_data;

    cmd_data.linear.x = 0;
    cmd_data.linear.y = 0;
    cmd_data.linear.z = 0;
    
    cmd_data.angular.x = 0;
    cmd_data.angular.y = 0;
    cmd_data.angular.z = 0;

    std::lock_guard<std::mutex> lock(vehicle_mtx);
    vehicle_cmd_data = cmd_data;
}


void FakeController::vehicle_cmd_circle(const double &line_speed, const double &angular_speed) {
    geometry_msgs::Twist cmd_data;

    cmd_data.linear.x = line_speed;
    cmd_data.linear.y = 0;
    cmd_data.linear.z = 0;
    
    cmd_data.angular.x = 0;
    cmd_data.angular.y = 0;
    cmd_data.angular.z = angular_speed;

    std::lock_guard<std::mutex> lock(vehicle_mtx);
    vehicle_cmd_data = cmd_data;
}

void FakeController::drone_cmd_circle(const double &line_speed, const double &angular_speed) {
    geometry_msgs::Twist cmd_data;
    if (1) {
        double cos_theta = sin(2 * M_PI * current_drone_pub_num / max_pub_num);
        double sin_theta = - cos(2 * M_PI * current_drone_pub_num / max_pub_num);

        cmd_data.linear.x = line_speed * cos_theta;
        cmd_data.linear.y = line_speed * sin_theta;
        cmd_data.linear.z = 0;
        
        cmd_data.angular.x = 0;
        cmd_data.angular.y = 0;
        cmd_data.angular.z = 0;
    }

    if (0) {
        cmd_data.linear.x = line_speed;
        cmd_data.linear.y = 0;
        cmd_data.linear.z = 0;
        
        cmd_data.angular.x = 0;
        cmd_data.angular.y = 0;
        cmd_data.angular.z = angular_speed;
    }
    std::lock_guard<std::mutex> lock(drone_mtx);
    drone_cmd_data = cmd_data;
}

void FakeController::drone_cmd_line(const double &speed) {
    geometry_msgs::Twist cmd_data;

    cmd_data.linear.x = speed;
    cmd_data.linear.y = 0;
    cmd_data.linear.z = 0;
    
    cmd_data.angular.x = 0;
    cmd_data.angular.y = 0;
    cmd_data.angular.z = 0;

    std::lock_guard<std::mutex> lock(drone_mtx);
    drone_cmd_data = cmd_data;
    current_drone_pub_num = 0;
}

void FakeController::vehicle_cmd_line(const double &speed) {
    geometry_msgs::Twist cmd_data;

    cmd_data.linear.x = speed;
    cmd_data.linear.y = 0;
    cmd_data.linear.z = 0;
    
    cmd_data.angular.x = 0;
    cmd_data.angular.y = 0;
    cmd_data.angular.z = 0;

    std::lock_guard<std::mutex> lock(vehicle_mtx);
    vehicle_cmd_data = cmd_data;
    current_vehicle_pub_num = 0;
}


void FakeController::pub_drone_control(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(drone_mtx);
    // 1. speed

    // 2. odom
    double dt = 0.1; 
    double delta_x = (drone_cmd_data.linear.x * cos(drone_theta) 
                    - drone_cmd_data.linear.y * sin(drone_theta)) * dt;
    double delta_y = (drone_cmd_data.linear.x * sin(drone_theta) 
                    + drone_cmd_data.linear.y * cos(drone_theta)) * dt;
    double delta_theta = drone_cmd_data.angular.z * dt;          

    drone_x += delta_x;
    drone_y -= delta_y;
    drone_theta += delta_theta;

    if (drone_theta > 2 * M_PI) drone_theta -= 2 * M_PI;
    if (drone_theta < 0) drone_theta += 2 * M_PI;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = drone_x;
    odom_msg.pose.pose.position.y = drone_y;
    odom_msg.pose.pose.position.z = 0.0;


    tf::Quaternion q = tf::createQuaternionFromYaw(drone_theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = drone_cmd_data.linear.x;
    odom_msg.twist.twist.angular.z = drone_cmd_data.angular.z;

    // 3. path
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x = drone_x;
    pose_stamped.pose.position.y = drone_y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(drone_theta);

    if (drone_path_points.size() >= 30) {
        drone_path_points.pop_front();
    }
    drone_path_points.push_back(pose_stamped);
    drone_path_msg.poses.clear();  
    for (const auto& point : drone_path_points) {
        drone_path_msg.poses.push_back(point);  
    }

    drone_rc_pub.publish(drone_cmd_data);
    drone_odom_pub.publish(odom_msg);
    drone_path_pub.publish(drone_path_msg);

    current_drone_pub_num++;
}

void FakeController::pub_vehicle_control(const ros::TimerEvent& event) {


    std::lock_guard<std::mutex> lock(vehicle_mtx);
    // 1. speed

    // 2. odom
    double dt = 0.1; 
    double delta_x = (vehicle_cmd_data.linear.x * cos(vehicle_theta) 
                    - vehicle_cmd_data.linear.y * sin(vehicle_theta)) * dt;
    double delta_y = (vehicle_cmd_data.linear.x * sin(vehicle_theta) 
                    + vehicle_cmd_data.linear.y * cos(vehicle_theta)) * dt;
    double delta_theta = vehicle_cmd_data.angular.z * dt;          

    vehicle_x += delta_x;
    vehicle_y += delta_y;
    vehicle_theta += delta_theta;

    if (vehicle_theta > 2 * M_PI) vehicle_theta -= 2 * M_PI;
    if (vehicle_theta < 0) vehicle_theta += 2 * M_PI;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "vehicle_base_link";

    odom_msg.pose.pose.position.x = vehicle_x;
    odom_msg.pose.pose.position.y = vehicle_y;
    odom_msg.pose.pose.position.z = 0.0;


    tf::Quaternion q = tf::createQuaternionFromYaw(vehicle_theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vehicle_cmd_data.linear.x;
    odom_msg.twist.twist.angular.z = vehicle_cmd_data.angular.z;

    // 3. path
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x = vehicle_x;
    pose_stamped.pose.position.y = vehicle_y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(vehicle_theta);

    if (vehicle_path_points.size() >= 30) {
        vehicle_path_points.pop_front();
    }
    vehicle_path_points.push_back(pose_stamped);
    vehicle_path_msg.poses.clear();  
    for (const auto& point : vehicle_path_points) {
        vehicle_path_msg.poses.push_back(point);  
    }

    vehicle_cmd_vel_pub.publish(vehicle_cmd_data);
    vehicle_odom_pub.publish(odom_msg);
    vehicle_path_pub.publish(vehicle_path_msg);

    current_vehicle_pub_num++;
}
