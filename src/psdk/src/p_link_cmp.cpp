#include <iostream>
#include <iomanip>  // For std::setw and std::setfill
#include "p_link_cmp.h"


p_link_cmp::p_link_cmp(/* args */)
{
    uav_data_update_flag = 0;
    last_uav_data_update_flag = 0;

    camer_track_update_flag = 0;
    last_camer_track_update_flag = 0;
}

p_link_cmp::~p_link_cmp()
{
}

void test(uint8_t *buf)
{

    return;
}

int p_link_cmp::p_link_cmp_init(int (*recv)(uint8_t *data, uint16_t length), int (*send)(uint8_t *data, uint16_t length))
{
    if(nullptr == recv || nullptr == send)
    {
        return -1;
    }

    recv_data = recv; //接收数据回调函数
    send_data = send; //发送回调函数

    msg_fun_register(P_LINK_UAV_DATA_MSG_ID,std::bind([this](uint8_t *buf){dataAnalysis(p_link_uav_data,buf); uav_data_update_flag++; },std::placeholders::_1)); //无人机数据解析注册
    msg_fun_register(P_LINK_UAV_CTRL_MSG_ID,std::bind([this](uint8_t *buf){dataAnalysis(UavCtrCmd,buf);},std::placeholders::_1)); //无人机控制数据解析注册
    msg_fun_register(P_LINK_GPS_TRACK_MSG_ID,std::bind([this](uint8_t *buf){dataAnalysis(p_link_gps_track,buf);},std::placeholders::_1)); //
    msg_fun_register(P_LINK_CAMERA_TRACK_MSG_ID,std::bind([this](uint8_t *buf){dataAnalysis(p_link_camer_track,buf);camer_track_update_flag++;},std::placeholders::_1)); //

    //msg_fun_register(P_LINK_UAV_DATA_MSG_ID,[this](uint8_t *buf){dataAnalysis(p_link_uav_data,buf); uav_data_update_flag++; }); //无人机数据解析注册
    //msg_fun_register(P_LINK_UAV_CTRL_MSG_ID,[this](uint8_t *buf){dataAnalysis(UavCtrCmd,buf);}); //无人机控制数据解析注册
    //msg_fun_register(P_LINK_GPS_TRACK_MSG_ID,[this](uint8_t *buf){dataAnalysis(p_link_gps_track,buf);}); //
    //msg_fun_register(P_LINK_CAMERA_TRACK_MSG_ID,[this](uint8_t *buf){dataAnalysis(p_link_camer_track,buf);camer_track_update_flag++;}); //
    //msg_fun_register(P_LINK_CAMERA_TRACK_MSG_ID,test); //
    return 0;
}

void p_link_cmp::printHexArray(const unsigned char* arr, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        // Print each byte in 2-digit hexadecimal format
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(arr[i]) << ' ';
    }
    std::cout << std::endl;
}

void p_link_cmp::run()
{
    uint8_t buf[300];
    int ret = recv_data(buf,300);
    
    if(ret > 0)
    {
        //printHexArray(buf, ret);
        //std::cout<<"read len:"<<ret<<std::endl;
        unpack(buf,ret);
    }
}

/**************************************************************************************************************************************************************************/
/**************************************************************************************************************************************************************************/
//发送无人机数据
int p_link_cmp::send_uav_data(T_p_link_uav_data *uav_data)
{
    uint8_t buf[256];
    uint8_t length = 0; 
    pack(P_LINK_UAV_DATA_MSG_ID,(uint8_t *)uav_data,sizeof(T_p_link_uav_data ),buf,&length);
    send_data(buf,length);
    return 0;
}

//相机框点数据
int p_link_cmp::camer_track_data(T_p_link_camer_track *camer_track_data)
{
    uint8_t buf[256];
    uint8_t length = 0; 
    pack(P_LINK_CAMERA_TRACK_MSG_ID,(uint8_t *)camer_track_data,sizeof(T_p_link_camer_track),buf,&length);
    send_data(buf,length);
    return 0;
}

//控制无人机飞行指令
int p_link_cmp::uav_ctrl_cmd(T_uav_ctrl uav_ctr_cmd)
{
    uint8_t buf[20];
    uint8_t length = 0; 
    T_uav_ctrl uav_ctr_cmd_temp = uav_ctr_cmd;
    printf("\r\n CMD[%d]\r\n",uav_ctr_cmd_temp.CMD);
    pack(P_LINK_UAV_CTRL_MSG_ID,(uint8_t *)&uav_ctr_cmd_temp,sizeof(uav_ctr_cmd_temp),buf,&length);
    send_data(buf,length);
    printHexArray(buf, length);
    return 0;
}


//控制无人机gps 跟随
int p_link_cmp::uav_follow_ctrl(T_p_link_gps_track *gps_track)
{
    uint8_t buf[50];
    uint8_t length = 0; 
    pack(P_LINK_GPS_TRACK_MSG_ID,(uint8_t *)gps_track,sizeof(T_p_link_gps_track),buf,&length);
    send_data(buf,length);
    return 0;
}

//rc 控制
int p_link_cmp::rc(T_p_link_rc *rc)
{
    uint8_t buf[50];
    uint8_t length = 0; 
    pack(P_LINK_RC_MSG_ID,(uint8_t *)rc,sizeof(T_p_link_gps_track),buf,&length);
    send_data(buf,length);
    return 0;
}

//uav data
int p_link_cmp::uav_data_get(T_p_link_uav_data *uav_data)
{
    if(nullptr == uav_data)
    {
        return -1;
    }

    if(uav_data_update_flag == last_uav_data_update_flag)
    {
        return 1;
    }
    last_uav_data_update_flag = uav_data_update_flag;
    *uav_data = p_link_uav_data;
    return 0;
}

//uav data
int p_link_cmp::camer_track_data_get(T_p_link_camer_track *camer_track_data)
{
    if(nullptr == camer_track_data)
    {
        return -1;
    }

    if(camer_track_update_flag == last_camer_track_update_flag )
    {
        return 1;
    }
    last_camer_track_update_flag = camer_track_update_flag;
    *camer_track_data = p_link_camer_track;
    return 0;
}

/**************************************************************************************************************************************************************************/
/**************************************************************************************************************************************************************************/
