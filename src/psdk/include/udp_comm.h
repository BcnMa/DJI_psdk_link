#ifndef UDP_COMM_H 
#define UDP_COMM_H 

#include <string>
#include <netinet/in.h>   // 包含 sockaddr_in 定义
#include <sys/socket.h>   // 包含 socket 函数定义


class udp_comm{
public:
    udp_comm(const std::string &ip, int port,int type);
    ~udp_comm();

    bool start();
    void stop();
    int send_data(uint8_t *data, uint16_t length);
    int recv_data(uint8_t *data, uint16_t length);

private:
    int sockfd;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;
    bool running;
    std::string serverIp;
    int serverPort;
    int server_type;

    std::string clientIp; 
    int clientPort; 

    void initServer();
    void closeSocket();
};

#endif // UDPSERVER_H
