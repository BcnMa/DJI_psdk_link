# include "psdk_node.h"

T_DjiUartHandle p_link_uartHandle;

int uart_send(uint8_t *data, uint16_t length)
{
    if (nullptr == data)
    {
        return -1;
    }
    uint32_t realLen = 0;

    HalUart_WriteData(p_link_uartHandle, data, length, &realLen);
    // std::cout << "uart send data" << std::endl;
    return realLen;
}

int uart_recv(uint8_t *data, uint16_t length)
{
    if (nullptr == data)
    {
        return -1;
    }
    uint32_t realLen = 0;
    HalUart_ReadData(p_link_uartHandle, data, length, &realLen);

    return realLen;
}


void pub_pose(p_link_cmp link_cmp, T_p_link_uav_data uav_data)
{
    if (0 == link_cmp.uav_data_get(&uav_data))
    {
        geometry_msgs::Pose pose_data;

        pose_data.position.x = uav_data.x;
        pose_data.position.y = uav_data.y;
        pose_data.position.z = uav_data.z;

        pose_data.orientation.x = uav_data.q0;
        pose_data.orientation.y = uav_data.q1;
        pose_data.orientation.z = uav_data.q2;
        pose_data.orientation.w = uav_data.q3;

        // uav_pose_pub.publish(pose_data);
        
        std::cout <<"uav_data1: "<<uav_data.q0<< ", "<<uav_data.q1<< ", " <<uav_data.q2<<", " <<uav_data.q3<< std::endl;
        std::cout <<"uav_data2: "<<uav_data.x<< ", "<<uav_data.y<<", "<<uav_data.z<<std::endl;
    }
}

void uav_ctrl_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    std::cout << "uav_ctrl_callback" << std::endl;
    T_p_link_rc p_link_rc;
    p_link_rc.x = (msg->linear.x * 10);
    p_link_rc.y = (msg->linear.y * 10);
    p_link_rc.z = (msg->linear.z * 10);
    p_link_rc.yaw = (msg->angular.z * 10);

    // link_cmp.rc(&p_link_rc);
}

void uav_follow_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    std::cout << "uav_follow_callback" << std::endl;    
    T_p_link_gps_track gps_track;
    gps_track.lon = msg->longitude * 10000000;
    gps_track.lat = msg->latitude * 10000000;
    gps_track.alt = msg->altitude * 1000;
    gps_track.speed = 3.0;
    // link_cmp.uav_follow_ctrl(&gps_track);    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "psdk_node");
    ros::NodeHandle nh("~");
    int port_num = 0;
    p_link_cmp link_cmp;
    E_DjiHalUartNum uart_port;
    T_p_link_uav_data uav_data = {0};
    T_p_link_camer_track camer_track_data = {0};

    ros::param::get("port_num", port_num);

    ros::Publisher uav_pose_pub = nh.advertise<geometry_msgs::Pose>("uav_pose", 100000);
    ros::Publisher uav_imu_pub = nh.advertise<sensor_msgs::Imu>("uav_imu", 100000);
    ros::Subscriber uav_ctrl_sub = nh.subscribe("uav_ctrl", 10, uav_ctrl_callback);
    ros::Subscriber uav_follow_sub = nh.subscribe("uav_follow", 10, uav_follow_callback);
    
    if (port_num >= 0 && port_num < 3) { 
        uart_port = static_cast<E_DjiHalUartNum>(DJI_HAL_UART_NUM_0 + port_num);
    } else {
        ROS_ERROR("Invalid port number: %d", port_num);
        uart_port = DJI_HAL_UART_NUM_0; 
    }
    std::cout << "uart_port: " << uart_port << std::endl;

    // 57600
    if (0 != HalUart_Init(uart_port, 115200, &p_link_uartHandle))
    {
        std::cout << "uart open error" << std::endl;
    }
    link_cmp.p_link_cmp_init(uart_recv, uart_send);

    while (1)
    {
        link_cmp.run();

        int cmd = 0;
        std::cout << "<1>起飞" << std::endl;
        std::cout << "<2>降落" << std::endl;
        std::cout << "<3>返航" << std::endl;
        std::cout << "<4>跟随飞行" << std::endl;
        std::cout << "<5>RC" << std::endl;

        std::cin >> cmd;

        switch (cmd)
        {
        case 1: // 启飞
        {
            double alt = 0;
            std::cout << "起飞高度:";
            std::cin >> alt;
            T_uav_ctrl uav_ctr_cmd;
            uav_ctr_cmd.CMD = E_UavCtr::TakeOff;
            uav_ctr_cmd.alt = alt;
            link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
        }
        break;

        case 2: // 降落
        {
            T_uav_ctrl uav_ctr_cmd;
            uav_ctr_cmd.CMD = E_UavCtr::Land;
            uav_ctr_cmd.alt = 0;
            link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
        }
        break;

        case 3: // 返航
        {
            T_uav_ctrl uav_ctr_cmd;
            uav_ctr_cmd.CMD = E_UavCtr::GoHome;
            uav_ctr_cmd.alt = 0;
            link_cmp.uav_ctrl_cmd(uav_ctr_cmd);
        }
        break;

        case 4: // 跟随点控制指令
        {
            T_p_link_gps_track gps_track;
            double lon = 0, lat = 0, alt = 0, speed = 0;
            std::cout << "请输入坐标点(经纬度)以空格分开:";
            std::cin >> lon >> lat >> alt >> speed;
            gps_track.lon = lon * 10000000;
            gps_track.lat = lat * 10000000;
            gps_track.alt = alt * 1000;
            gps_track.speed = speed;
            link_cmp.uav_follow_ctrl(&gps_track);
        }
        break;

        case 5: // rc 控制
        {
            uint16_t send_count = 0;//RC 控制指令消息
            uint16_t send_hz = 0;//50 ms 发送一次
            T_p_link_rc p_link_rc;
            double x = 0, y = 0, z = 0, yaw = 0,tim = 0;
            std::cout << "请输入x,y,z,yaw,time控制速度,以空格分开:";
            std::cin >> x >> y >> z >> yaw >> tim;

            while(1)
            {
                send_count ++;
                send_hz++;
                if(send_count > (tim * 100)) //飞行控制控制10s
                {
                    break;
                }
                
                if(send_hz >= 10)
                {
                    send_hz = 0;
                    p_link_rc.x = (x * 10);
                    p_link_rc.y = (y * 10);
                    p_link_rc.z = (z * 10);
                    p_link_rc.yaw = (yaw * 10);

                    link_cmp.rc(&p_link_rc);
                }

        	usleep(10000);
            }

        }
        break;
        }
        usleep(10000);
    }

    return 0;
}

// 要在线程中执行的函数
void uav_data_recv_display_thread(const std::string &message)
{
    std::cout << message << std::endl;
}
