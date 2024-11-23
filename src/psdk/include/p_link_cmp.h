#ifndef __P_LINK_CMP_H
#define __P_LINK_CMP_H

#include<functional>

#include "p_link.h"

#define P_LINK_UAV_DATA_MSG_ID 1 //无人机数据消息

#define P_LINK_UAV_CTRL_MSG_ID 10 //无人机控制消息
#define P_LINK_GPS_TRACK_MSG_ID 11 //gps 跟踪消息指令
#define P_LINK_CAMERA_TRACK_MSG_ID 12 //相机 跟踪消息指令
#define P_LINK_RC_MSG_ID 13 //RC 控制指令消息


typedef struct  __attribute__((packed)) 
{
    float q0; /*!< w, rad (when converted to a rotation matrix or Euler angles). */
    float q1; /*!< x, rad (when converted to a rotation matrix or Euler angles). */
    float q2; /*!< y, rad (when converted to a rotation matrix or Euler angles). */
    float q3; /*!< z, rad (when converted to a rotation matrix or Euler angles). */
    int32_t x; /*!< Specifies int32 value of x for vector. */
    int32_t y; /*!< Specifies int32 value of y for vector. */
    int32_t z; /*!< Specifies int32 value of z for vector. */
}T_p_link_uav_data;

enum class E_UavCtr
{
    FC_IDLE = 0,
    TakeOff,
    Land,
    GoHome,
    FC_GUIDE
};

typedef struct  __attribute__((packed)) 
{
    E_UavCtr CMD;
    int32_t alt;
}T_uav_ctrl;

typedef struct  __attribute__((packed)) 
{
    int32_t lon; /*!< Specifies int32 value of x for vector. */
    int32_t lat; /*!< Specifies int32 value of y for vector. */
    int32_t alt; /*!< Specifies int32 value of z for vector. */
    int32_t speed;
}T_p_link_gps_track;

typedef struct __attribute__((packed))
{
    T_p_link_gps_track poin1;
    T_p_link_gps_track poin2;
    T_p_link_gps_track poin3;
} T_p_link_camer_track;

typedef struct __attribute__((packed))
{
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t yaw;
} T_p_link_rc;

/**********************************************************************************************/

class p_link_cmp : public p_link
{
private:
    int (*recv_data)(uint8_t *data, uint16_t length);
    int (*send_data)(uint8_t *data, uint16_t length);

    E_UavCtr UavCtrCmd;
    T_p_link_uav_data p_link_uav_data;
    T_p_link_gps_track p_link_gps_track;
    T_p_link_camer_track p_link_camer_track;

    uint8_t uav_data_update_flag;
    uint8_t last_uav_data_update_flag;

    uint8_t camer_track_update_flag;
    uint8_t last_camer_track_update_flag;

    void printHexArray(const unsigned char *arr, size_t size);

    /* data */
public:
    p_link_cmp(/* args */);
    ~p_link_cmp();

    int p_link_cmp_init(int (*recv)(uint8_t *data, uint16_t length), int (*send)(uint8_t *data, uint16_t length));
    void run();

    int send_uav_data(T_p_link_uav_data *uav_data);
    int camer_track_data(T_p_link_camer_track *camer_track_data);
    int uav_data_get(T_p_link_uav_data *uav_data);
    int camer_track_data_get(T_p_link_camer_track *camer_track_data);
    int uav_ctrl_cmd(T_uav_ctrl uav_ctr_cmd);
    int uav_follow_ctrl(T_p_link_gps_track *gps_track);
    int rc(T_p_link_rc *rc);
};

#endif
