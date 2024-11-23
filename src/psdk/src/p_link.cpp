#include <iostream>
#include "p_link.h"

p_link::p_link(/* args */)
{
    seq = 0;
}

p_link::~p_link()
{
}

int p_link::msg_fun_register(uint8_t msgId, msg_func cb)
{
    msg_cb_fun_map.emplace(msgId, cb);

    return 0;
}

//template <typename _T>
//void p_link::dataAnalysis(_T &target, uint8_t *source)
//{
//    target = *(_T *)source;
//}

// 求和校验
uint8_t p_link::sum_check(uint8_t *data, uint16_t length)
{
    uint8_t sum = 0;
    if (nullptr == data)
    {
        return 0;
    }

    for (int i = 0; i < length; i++)
    {
        sum += data[i];
    }

    return sum;
}

// @brief  数据打包
// @param msgId 
// @param data 
// @param length 
// @param out_data 
// @param out_length 
// @return 
int p_link::pack(uint8_t msgId, uint8_t *data, uint8_t length, uint8_t *out_data, uint8_t *out_length)
{
    if (nullptr == data || nullptr == out_data)
    {
        return -1;
    }
    uint8_t index = 0;
    out_data[index++] = P_LINK_HEAD;
    out_data[index++] = length;
    out_data[index++] = seq++;
    out_data[index++] = msgId;
    memcpy(&out_data[index], data, length);
    index += length;
    uint8_t sum = sum_check(out_data, index);
    out_data[index++] = sum;
    *out_length = index;
    return 0;
}


//消息解析
// @brief 
// @param data 
// @param length 
// @return 
int p_link::unpack(uint8_t *data, uint8_t length)
{
    int ret = 0;
    if (nullptr == data)
    {
        return -1;
    }

    // p_link_r_data.buf;
    for (uint16_t i = 0; i < length; i++)
    {
        switch (link_buf.state)
        {
        case 0: // 查找头
            if (P_LINK_HEAD == data[i])
            {
                link_buf.data[0] = P_LINK_HEAD;
                link_buf.read_index++;
                link_buf.state = 1;
                //std::cout << "setp 1" << std::endl;
            }
            break;

        case 1: // 获取数据长度
            link_buf.data[link_buf.read_index++] = data[i];
            link_buf.pack_length = data[i] + 5;
            link_buf.state = 2;
            //std::cout<<"setp 2"<<std::endl;
            break;
        case 2:
            link_buf.data[link_buf.read_index++] = data[i];
            if (link_buf.read_index >= link_buf.pack_length)
            {
                //std::cout<<"setp 3"<<std::endl;
                // 数据解析处理
                // rs_link_msg_process(link_buf.data, link_buf.pack_length);
                //std::cout<<data[0]<<std::endl;
                auto it = msg_cb_fun_map.find(link_buf.data[P_LINK_MSG_ID_INDEX]); // 查找键为2的元素
                if (it != msg_cb_fun_map.end())
                {
                    it->second(&link_buf.data[P_LINK_PAYLOAD_INDEX]);
                }
                link_buf.read_index = 0;
                link_buf.pack_length = 0;
                link_buf.state = 0;
            }
            break;
        }
    }
    return ret;
}
