#include "udp_comm.h"
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>

#include <fcntl.h>  // 包含 fcntl 函数


udp_comm::udp_comm(const std::string& ip, int port,int type)
    : serverIp(ip), serverPort(port),server_type(type), running(false), sockfd(-1) {
    memset(&serverAddr, 0, sizeof(serverAddr));
    memset(&clientAddr, 0, sizeof(clientAddr));
}

udp_comm::~udp_comm() {
    stop();
}

#if 0
void udp_comm::initClient() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket!" << std::endl;
        exit(EXIT_FAILURE);
    }



    std::memset(&clientAddr, 0, sizeof(clientAddr));
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(CLIENT_PORT);  // 指定源端口
    clientAddr.sin_addr.s_addr = INADDR_ANY;   // 绑定到所有本地网络接口


    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    serverAddr.sin_addr.s_addr = inet_addr(serverIp.c_str());

    if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::cerr << "Bind failed!" << std::endl;
        closeSocket();
        exit(EXIT_FAILURE);
    }
    // 设置套接字为非阻塞模式
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1)
    {
        std::cerr << "Failed to get socket flags!" << std::endl;
        closeSocket();
        exit(EXIT_FAILURE);
    }

    flags |= O_NONBLOCK; // 设置 O_NONBLOCK 标志
    if (fcntl(sockfd, F_SETFL, flags) == -1)
    {
        std::cerr << "Failed to set non-blocking mode!" << std::endl;
        closeSocket();
        exit(EXIT_FAILURE);
    }

    running = true;
    std::cout << "Server started on " << serverIp << ":" << serverPort << std::endl;
}
#endif


void udp_comm::initServer() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket!" << std::endl;
        exit(EXIT_FAILURE);
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    serverAddr.sin_addr.s_addr = inet_addr(serverIp.c_str());

    if(0 == server_type)
    {
        if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
        {
            std::cerr << "Bind failed!" << std::endl;
            closeSocket();
            exit(EXIT_FAILURE);
        }
    }

    // 设置套接字为非阻塞模式
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1)
    {
        std::cerr << "Failed to get socket flags!" << std::endl;
        closeSocket();
        exit(EXIT_FAILURE);
    }

    flags |= O_NONBLOCK; // 设置 O_NONBLOCK 标志
    if (fcntl(sockfd, F_SETFL, flags) == -1)
    {
        std::cerr << "Failed to set non-blocking mode!" << std::endl;
        closeSocket();
        exit(EXIT_FAILURE);
    }

    running = true;
    std::cout << "Server started on " << serverIp << ":" << serverPort << std::endl;
}

void udp_comm::closeSocket()
{
    if (sockfd != -1)
    {
        close(sockfd);
        sockfd = -1;
    }
}

bool udp_comm::start()
{
    if (!running)
    {
        initServer();
        send_data((uint8_t *)"connect", 6);
        return true;
    }
    return false;
}

void udp_comm::stop()
{
    if (running)
    {
        closeSocket();
        running = false;
        std::cout << "Server stopped." << std::endl;
    }
}

int udp_comm::send_data(uint8_t *data, uint16_t length)
{
    if (!running)
    {
        std::cerr << "Server is not running!" << std::endl;
        return -1;
    }

    ssize_t sentBytes  = 0;

    if(0 == server_type)
    {
        struct sockaddr_in targetAddr;
        memset(&targetAddr, 0, sizeof(targetAddr));

        targetAddr.sin_family = AF_INET;
        targetAddr.sin_port = htons(clientPort);
        targetAddr.sin_addr.s_addr = inet_addr(clientIp.c_str());

        sentBytes = sendto(sockfd, data, length, 0, (struct sockaddr *)&targetAddr, sizeof(targetAddr));

        if (sentBytes < 0)
        {
            std::cerr << "Failed to send message!" << std::endl;
        }
        // std::cout << "Sent message: " << message << std::endl;
    }
    else
    {
        sentBytes = sendto(sockfd, data, length, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

        if (sentBytes < 0)
        {
            std::cerr << "Failed to send message!" << std::endl;
        }
        // std::cout << "Sent message: " << message << std::endl;
    }
    return sentBytes;
}

int udp_comm::recv_data(uint8_t *data, uint16_t length)
{
    if (!running)
    {
        std::cerr << "Server is not running!" << std::endl;
        return 0;
    }

    socklen_t addrLen = sizeof(clientAddr);
    ssize_t recvBytes = recvfrom(sockfd, data, length, 0, (struct sockaddr *)&clientAddr, &addrLen);

    if (recvBytes < 0)
    {
        // std::cerr << "Failed to receive message!" << std::endl;
        return recvBytes;
    }
    // buffer[recvBytes] = '\0';  // Null terminate the received string
    // std::string receivedMsg(buffer);
    // std::cout << "Received message: " << receivedMsg << std::endl;

    // Here you can use clientAddr to send a reply back to the client.
    clientIp = inet_ntoa(clientAddr.sin_addr);
    clientPort = ntohs(clientAddr.sin_port);

    std::cout << "Client's IP: " << clientIp << ", Port: " << clientPort << std::endl;

    return recvBytes;
}
