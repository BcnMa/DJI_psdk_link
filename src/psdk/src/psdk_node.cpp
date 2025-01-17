# include "psdk_node.hpp"
# include <signal.h>
# include <cstdlib>

void sigint_handler(int sig) {
    if (sig == SIGINT) {
        std::cout << "Ctrl+C pressed!" << std::endl;
        std::exit(0); 
    }   
}

udp_comm Psdk::udp_client("127.0.0.1", 6868, 1); // 在 .cpp 文件中初始化静态成员

int main(int argc, char *argv[])
{
    // signal(SIGINT, sigint_handler);

    ros::init(argc, argv, "psdk_node");
    ros::NodeHandle nh("~");

    Psdk psdk;
    

    psdk.uart_init();

    // ctrl_mode = 0：run_only_pc，电脑键盘输入控制
    // ctrl_mode = 1：    run_ros，通过ros话题控制
    int ctrl_mode;
    nh.param<int>("/ctrl_mode", ctrl_mode, 0);
    if (ctrl_mode == 0) {
        psdk.run_only_pc();
    } else {
        psdk.run_ros();
    }

    return 0;
}

