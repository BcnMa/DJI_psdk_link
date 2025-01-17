# include "ros/ros.h"
# include "geometry_msgs/Twist.h"
# include "geometry_msgs/Pose.h"
# include "sensor_msgs/NavSatFix.h"
# include "sensor_msgs/Imu.h"
# include "psdk_msgs/DroneState.h"

# include "iostream"
# include "mutex"
# include "thread"
# include "signal.h"
# include "p_link_cmp.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "hal_uart.h"
#ifdef __cplusplus
}
#endif

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

T_DjiUartHandle p_link_uartHandle;

enum DroneState {
    IDLE = 0,
    LAUNCHING = 1,  // 起飞
    LANDING = 2,    // 降落
    BACKING = 3,    // 返航
    POINT = 4,      // 点控
    RC = 5          // RC控制
};


class Psdk {
private:

    bool debug_mode;
    bool has_launched = false;
    bool has_landed = false;
    bool has_backed = false;
    int port_num;
    
    p_link_cmp link_cmp;
    E_DjiHalUartNum uart_port;
    T_p_link_uav_data uav_data = {0};
    T_p_link_camer_track camer_track_data = {0};
    DroneState drone_state = DroneState::IDLE;

    ros::Publisher drone_pose_pub; 
    ros::Publisher drone_imu_pub;
    ros::Subscriber drone_ctrl_sub;
    ros::Subscriber drone_follow_sub;
    ros::Subscriber drone_state_sub;
    ros::Timer timer_drone_ctrl;

    std::mutex mtx;

public:
    Psdk();
    ~Psdk() = default;

    void drone_ctrl_timer_callback(const ros::TimerEvent &event);
    void drone_ctrl_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void drone_follow_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void drone_state_callback(const psdk_msgs::DroneState::ConstPtr &msg);
    void pub_pose();

    int uart_init();
    static int uart_send(uint8_t *data, uint16_t length);
    static int uart_recv(uint8_t *data, uint16_t length);

    void run_ros();
    void run_only_pc();
}; // class Psdk


Psdk::Psdk() {
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    drone_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("/uav_pose", 100000);
    drone_imu_pub = nh.advertise<sensor_msgs::Imu>("/uav_imu", 100000);
    drone_ctrl_sub = nh.subscribe("/rc_ctrl", 10, &Psdk::drone_ctrl_callback, this);
    drone_follow_sub = nh.subscribe("/pose_follow", 10, &Psdk::drone_follow_callback, this);
    drone_state_sub = nh.subscribe("/drone_state", 10, &Psdk::drone_state_callback, this);
    timer_drone_ctrl = nh.createTimer(ros::Duration(0.1), &Psdk::drone_ctrl_timer_callback, this);
    nh_private.param<int>("port_num", port_num, 0);
    nh_private.param<bool>("debug_mode", debug_mode, false);
}

void Psdk::pub_pose() {
    sensor_msgs::NavSatFix pose_data;
    if (0 == link_cmp.uav_data_get(&uav_data)) {
        pose_data.header.stamp = ros::Time::now();
        pose_data.header.frame_id = "uav";

        pose_data.longitude = uav_data.x / 10000000.0;
        pose_data.latitude = uav_data.y / 10000000.0;
        pose_data.altitude = uav_data.z / 1000.0;

        drone_pose_pub.publish(pose_data);
        if (debug_mode) 
            std::cout << CYAN << std::endl << "The Drone send state:" << RED << " Position " << YELLOW << std::endl
                    << "    x: " << std::fixed << std::setprecision(6) << pose_data.longitude << ", " << std::endl
                    << "    y: " << std::fixed << std::setprecision(6) << pose_data.latitude << ", " << std::endl
                    << "    z: " << std::fixed << std::setprecision(6) << pose_data.altitude << ", " << std::endl
                    << "    q0: " << std::fixed << std::setprecision(6) << uav_data.q0 << ", " << std::endl 
                    << "    q1: " << std::fixed << std::setprecision(6) << uav_data.q1 << ", " << std::endl
                    << "    q2: " << std::fixed << std::setprecision(6) << uav_data.q2 << ", " << std::endl 
                    << "    q3: " << std::fixed << std::setprecision(6) << uav_data.q3 
                    << RESET << std::endl;  
    } 
}

void Psdk::drone_ctrl_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx);
    if (0) // debug_mode
        std::cout << GREEN << std::endl << "The Drone receives command:" << RED << " Speed" << YELLOW << std::endl
                    << "    x-speed: " << msg->linear.x << ", " << std::endl 
                    << "    y-speed: " << msg->linear.y << ", " << std::endl
                    << "    z-speed: " << msg->linear.z << ", " << std::endl
                    << "    yaw-speed: " << msg->angular.z << RESET
                    << std::endl;
    T_p_link_rc p_link_rc;
    p_link_rc.x = (msg->linear.x * 10);
    p_link_rc.y = (msg->linear.y * 10);
    p_link_rc.z = (msg->linear.z * 10);
    p_link_rc.yaw = (msg->angular.z * 10);
    if (drone_state == DroneState::RC)
        link_cmp.rc(&p_link_rc);
}

void Psdk::drone_follow_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx);
    if (debug_mode)
        std::cout << GREEN << std::endl << "The Drone receives command:" << RED << " Position " << YELLOW << std::endl
                    << "    longitude: " << std::fixed << std::setprecision(6) << msg->longitude << ", " << std::endl
                    << "    latitude: " << std::fixed << std::setprecision(6) << msg->latitude << ", " << std::endl
                    << "    altitude: " << std::fixed << std::setprecision(6) << msg->altitude << ", " << RESET
                    << std::endl;    
    T_p_link_gps_track gps_track;
    gps_track.lon = msg->longitude * 10000000;
    gps_track.lat = msg->latitude * 10000000;
    gps_track.alt = msg->altitude * 1000;
    gps_track.speed = 3.0;
    if (drone_state == DroneState::POINT)
        link_cmp.uav_follow_ctrl(&gps_track);    
}

void Psdk::drone_state_callback(const psdk_msgs::DroneState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx);


    switch (msg->state) {
        case 0:
            drone_state = DroneState::IDLE;
            break;
        case 1:
            drone_state = DroneState::LAUNCHING;
            break;
        case 2:
            drone_state = DroneState::LANDING;
            break;
        case 3:
            drone_state = DroneState::BACKING;
            break;
        case 4:
            drone_state = DroneState::POINT;
            break;
        case 5:
            drone_state = DroneState::RC;
            break;
        default:
            break;
    }
}

void Psdk::drone_ctrl_timer_callback(const ros::TimerEvent &event) {
    std::lock_guard<std::mutex> lock(mtx);
    if (drone_state == DroneState::LAUNCHING
        && !has_launched) {
        T_uav_ctrl uav_ctr_cmd;
        uav_ctr_cmd.CMD = E_UavCtr::TakeOff;
        uav_ctr_cmd.alt = 10.0;
        link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
        has_launched = true;
        std::cout << GREEN << "Now launching to " 
            << RED << static_cast<double>(uav_ctr_cmd.alt) 
            << GREEN << " meter." << RESET << std::endl;
    }

    if (drone_state == DroneState::LANDING
        && !has_landed) {
        T_uav_ctrl uav_ctr_cmd;
        uav_ctr_cmd.CMD = E_UavCtr::Land;
        uav_ctr_cmd.alt = 0;
        link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
        has_landed = true;
        std::cout << GREEN << "Now landing " << RESET << std::endl;
    }

    if (drone_state == DroneState::BACKING
        && !has_backed) {
        T_uav_ctrl uav_ctr_cmd;
        uav_ctr_cmd.CMD = E_UavCtr::GoHome;
        uav_ctr_cmd.alt = 0;
        link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
        has_backed = true;
        std::cout << GREEN << "Now backing " << RESET << std::endl;
    }
}

int Psdk::uart_init() {
    if (port_num >= 0 && port_num < 3) { 
        uart_port = static_cast<E_DjiHalUartNum>(DJI_HAL_UART_NUM_0 + port_num);
    } else {
        ROS_ERROR("Invalid port number: %d", port_num);
        uart_port = DJI_HAL_UART_NUM_0; 
    }
    std::cout << "uart_port: " << uart_port << std::endl;

    // 57600
    if (0 != HalUart_Init(uart_port, 115200, &p_link_uartHandle)) {
        std::cout << "uart open error" << std::endl;
    }
    link_cmp.p_link_cmp_init(uart_recv, uart_send);

    if (debug_mode) {
        std::cout << GREEN << "ros param:" << YELLOW << std::endl
                    << "    port_num: " << port_num << std::endl
                    << "    debug_mode: " << debug_mode << RESET 
                    << std::endl;
    }
    return 0;
}

int Psdk::uart_send(uint8_t *data, uint16_t length) {
    if (nullptr == data) {
        return -1;
    }
    uint32_t realLen = 0;

    HalUart_WriteData(p_link_uartHandle, data, length, &realLen);
    // std::cout << "uart send data" << std::endl;
    return realLen;
}

int Psdk::uart_recv(uint8_t *data, uint16_t length) {
    if (nullptr == data) {
        return -1;
    }
    uint32_t realLen = 0;
    HalUart_ReadData(p_link_uartHandle, data, length, &realLen);

    return realLen;
}

void Psdk::run_ros() {
    ros::Rate rate(100); 

    while (ros::ok()) {
        link_cmp.run();
        pub_pose();

        ros::spinOnce();
        rate.sleep();
    }
}

void Psdk::run_only_pc() {
    while (1) {
        pub_pose();
        link_cmp.run();

        int cmd = 0;
        std::cout << "<0>退出" << std::endl;
        std::cout << "<1>起飞" << std::endl;
        std::cout << "<2>降落" << std::endl;
        std::cout << "<3>返航" << std::endl;
        std::cout << "<4>跟随飞行" << std::endl;
        std::cout << "<5>RC" << std::endl;

        std::cin >> cmd;

        switch (cmd) {
            case 0: { // 退出
                exit(0);
            }
            case 1: { // 启飞
                double alt = 0;
                std::cout << "起飞高度:";
                std::cin >> alt;
                T_uav_ctrl uav_ctr_cmd;
                uav_ctr_cmd.CMD = E_UavCtr::TakeOff;
                uav_ctr_cmd.alt = alt;
                link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
            } break;

            case 2: { // 降落
                T_uav_ctrl uav_ctr_cmd;
                uav_ctr_cmd.CMD = E_UavCtr::Land;
                uav_ctr_cmd.alt = 0;
                link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
            } break;

            case 3: { // 返航
                T_uav_ctrl uav_ctr_cmd;
                uav_ctr_cmd.CMD = E_UavCtr::GoHome;
                uav_ctr_cmd.alt = 0;
                link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
            } break;

            case 4: { // 跟随点控制指令
                T_p_link_gps_track gps_track;
                double lon = 0, lat = 0, alt = 0, speed = 0;
                std::cout << "请输入坐标点(经纬度)以空格分开:";
                std::cin >> lon >> lat >> alt >> speed;
                gps_track.lon = lon * 10000000;
                gps_track.lat = lat * 10000000;
                gps_track.alt = alt * 1000;
                gps_track.speed = speed;
                link_cmp.uav_follow_ctrl(&gps_track);
            } break;

            case 5: { // rc 控制
                uint16_t send_count = 0;//RC 控制指令消息
                uint16_t send_hz = 0;//50 ms 发送一次
                T_p_link_rc p_link_rc;
                double x = 0, y = 0, z = 0, yaw = 0,tim = 0;
                std::cout << "请输入x,y,z,yaw,time控制速度,以空格分开:";
                std::cin >> x >> y >> z >> yaw >> tim;

                while(1) {
                    send_count ++;
                    send_hz++;
                    if(send_count > (tim * 100)) //飞行控制控制10s
                    {
                        break;
                    }
                    
                    if(send_hz >= 10) {
                        send_hz = 0;
                        p_link_rc.x = (x * 10);
                        p_link_rc.y = (y * 10);
                        p_link_rc.z = (z * 10);
                        p_link_rc.yaw = (yaw * 10);

                        link_cmp.rc(&p_link_rc);
                    }
                    usleep(10000);
                } // while
            } break;
        } // switch
        usleep(10000);
    } // while
} 