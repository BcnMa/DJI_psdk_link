#ifndef __P_LINK_H
#define __P_LINK_H

#include <stdint.h>
#include <cstring> // For strcpy and strlen
#include <functional>
#include <map>

#define P_LINK_HEAD 0xFD

#define P_LINK_HEAD_INDEX 0
#define P_LINK_LEN_INDEX 1    //数据包长度
#define P_LINK_SEQ_INDEX 2    // 包序列号 
#define P_LINK_MSG_ID_INDEX 3 // 消息id 
#define P_LINK_PAYLOAD_INDEX 4

using msg_func = std::function<void(uint8_t *)>;

typedef struct
{
    uint8_t state;
    uint8_t pack_length;
    uint8_t read_index;
    uint8_t data[256];
} T_p_link_buf;

// template <typename _T>
class p_link
{
private:
    /* data */
    T_p_link_buf link_buf;
    std::map<uint8_t, msg_func> msg_cb_fun_map;
    uint8_t seq;

public:
    p_link(/* args */);
    ~p_link();

    uint8_t sum_check(uint8_t *data, uint16_t length);
    int pack(uint8_t msgId, uint8_t *data, uint8_t length, uint8_t *out_data, uint8_t *out_length);
    int unpack(uint8_t *data, uint8_t length);
    int msg_fun_register(uint8_t msgId, msg_func cb);

    template <typename _T>
    void dataAnalysis(_T &target, uint8_t *source)
    {
        target = *(_T *)source;
    }
};

#endif
