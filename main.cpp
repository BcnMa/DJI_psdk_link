#include <iostream>
#include <thread>

#include "link/p_link_cmp.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "hal/hal_uart.h"
#ifdef __cplusplus
}
#endif

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

int main(int argc, char **argv)
{
    p_link_cmp link_cmp;
    T_p_link_uav_data uav_data = {0};
    T_p_link_camer_track camer_track_data = {0};

    if (0 != HalUart_Init(DJI_HAL_UART_NUM_2, 57600, &p_link_uartHandle))
    {
        std::cout << "uart open error" << std::endl;
    }
    link_cmp.p_link_cmp_init(uart_recv, uart_send);

    while (1)
    {
        link_cmp.run();

#if 0
        if (0 == link_cmp.uav_data_get(&uav_data))
        {
            //无人机数据有更新，打印无人机数据
            //std::cout << uav_data.q0 << uav_data.q1 << uav_data.q2 << uav_data.q3 << std::endl;
            //std::cout << uav_data.x << uav_data.y << uav_data.z << std::endl;
            std::cout <<"uav_data1: "<<uav_data.q0<< ", "<<uav_data.q1<< ", " <<uav_data.q2<<", " <<uav_data.q3<< std::endl;
            std::cout <<"uav_data2: "<<uav_data.x<< ", "<<uav_data.y<<", "<<uav_data.z<<std::endl;
        }

        //相机识别矩形坐标数据
        if(0 == link_cmp.camer_track_data_get(&camer_track_data))
        {
            std::cout <<"camer_poin1: "<<camer_track_data.poin1.lat<< ", "<<camer_track_data.poin1.lon<< ", " <<camer_track_data.poin1.alt<<std::endl;
            std::cout <<"camer_poin2: "<<camer_track_data.poin2.lat<< ", "<<camer_track_data.poin2.lon<< ", " <<camer_track_data.poin2.alt<<std::endl;
            std::cout <<"camer_poin3: "<<camer_track_data.poin3.lat<< ", "<<camer_track_data.poin3.lon<< ", " <<camer_track_data.poin3.alt<<std::endl;
        }
#endif

#if 1
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
#endif
        usleep(10000);
    }

    return 0;
}

// 要在线程中执行的函数
void uav_data_recv_display_thread(const std::string &message)
{
    std::cout << message << std::endl;
}
