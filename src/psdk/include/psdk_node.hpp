# include "ros/ros.h"
# include "geometry_msgs/Twist.h"
# include "geometry_msgs/Pose.h"
# include "sensor_msgs/NavSatFix.h"
# include "sensor_msgs/Imu.h"

#include <iostream>
#include <thread>
#include <signal.h>
#include "p_link_cmp.h"

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

class Psdk {
private:
    ros::NodeHandle nh;
    bool debug_mode;
    int port_num;
    p_link_cmp link_cmp;
    E_DjiHalUartNum uart_port;
    T_p_link_uav_data uav_data = {0};
    T_p_link_camer_track camer_track_data = {0};
    ros::Publisher uav_pose_pub; 
    ros::Publisher uav_imu_pub;
    ros::Subscriber uav_ctrl_sub;
    ros::Subscriber uav_follow_sub;

public:
    Psdk(ros::NodeHandle &main_nh) {
        nh = main_nh;
        uav_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("/uav_pose", 100000);
        uav_imu_pub = nh.advertise<sensor_msgs::Imu>("/uav_imu", 100000);
        uav_ctrl_sub = nh.subscribe("/rc_ctrl", 10, &Psdk::uav_ctrl_callback, this);
        uav_follow_sub = nh.subscribe("/pose_follow", 10, &Psdk::uav_follow_callback, this);
    
        nh.param<int>("port_num", port_num, 0);
        nh.param<bool>("debug_mode", debug_mode, false);
    }

    static int uart_send(uint8_t *data, uint16_t length) {
        if (nullptr == data) {
            return -1;
        }
        uint32_t realLen = 0;

        HalUart_WriteData(p_link_uartHandle, data, length, &realLen);
        // std::cout << "uart send data" << std::endl;
        return realLen;
    }

    static int uart_recv(uint8_t *data, uint16_t length) {
        if (nullptr == data) {
            return -1;
        }
        uint32_t realLen = 0;
        HalUart_ReadData(p_link_uartHandle, data, length, &realLen);

        return realLen;
    }

    int uart_init() {
        if (port_num >= 0 && port_num < 3) { 
            uart_port = static_cast<E_DjiHalUartNum>(DJI_HAL_UART_NUM_0 + port_num);
        } else {
            ROS_ERROR("Invalid port number: %d", port_num);
            uart_port = DJI_HAL_UART_NUM_0; 
        }
        std::cout << "uart_port: " << uart_port << std::endl;

        // 57600
        if (0 != HalUart_Init(uart_port, 57600, &p_link_uartHandle)) {
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

    void run_ros() {
        ros::Rate rate(100); 

        while (ros::ok()) {
            link_cmp.run();
            pub_pose();

            ros::spinOnce();
            rate.sleep();
        }
    }

    void run_only_pc () {
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

    void pub_pose() {
        sensor_msgs::NavSatFix pose_data;
        if (0 == link_cmp.uav_data_get(&uav_data)) {
            pose_data.header.stamp = ros::Time::now();
            pose_data.header.frame_id = "uav";

            pose_data.longitude = uav_data.x / 10000000.0;
            pose_data.latitude = uav_data.y / 10000000.0;
            pose_data.altitude = uav_data.z / 1000.0;

            uav_pose_pub.publish(pose_data);
            if (debug_mode) 
                std::cout << CYAN << std::endl << "The Drone send state:" << RED << " Position " << YELLOW << std::endl
                        << "    x: " << uav_data.x << ", " << std::endl
                        << "    y: " << uav_data.y << ", " << std::endl
                        << "    z: " << uav_data.z << ", " << std::endl
                        << "    q0: " << uav_data.q0 << ", " << std::endl 
                        << "    q1: " << uav_data.q1 << ", " << std::endl
                        << "    q2: " << uav_data.q2 << ", " << std::endl 
                        << "    q3: " << uav_data.q3 << ", " << RESET
                        << std::endl;      
        } 
    }

    void uav_ctrl_callback(const geometry_msgs::Twist::ConstPtr &msg) {
        if (debug_mode)
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

        link_cmp.rc(&p_link_rc);
    }

    void uav_follow_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        if (debug_mode)
            std::cout << GREEN << std::endl << "The Drone receives command:" << RED << " Position " << YELLOW << std::endl
                        << "    longitude: " << msg->longitude << ", " << std::endl
                        << "    latitude: " << msg->latitude << ", " << std::endl
                        << "    altitude: " << msg->altitude << ", " << RESET
                        << std::endl;    
        T_p_link_gps_track gps_track;
        gps_track.lon = msg->longitude * 10000000;
        gps_track.lat = msg->latitude * 10000000;
        gps_track.alt = msg->altitude * 1000;
        gps_track.speed = 3.0;
        link_cmp.uav_follow_ctrl(&gps_track);    
    }

}; // class Psdk