# include "fake_controller.hpp"
# include "psdk_msgs/DroneState.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_controller"); 
    ros::NodeHandle nh("~"); 
    ros::Publisher drone_state_pub = nh.advertise<psdk_msgs::DroneState>("/drone_state", 100000);
    ros::Rate loop_rate(100);
    ros::Duration(1.0).sleep();

    double drone_line_speed = 3.0, drone_angular_speed = 10.0;
    double vehicle_line_speed = 0.5, vehicle_angular_speed = 1.0;

    FakeController fake_controller;
    psdk_msgs::DroneState drone_state_msg;
    drone_state_msg.state = 0;

    ros::Time start_time = ros::Time::now();

    bool stage_1_printed = false;
    bool stage_2_printed = false;
    bool stage_3_printed = false;
    bool stage_4_printed = false;
    bool stage_5_printed = false;
    bool stage_6_printed = false;
    bool stage_7_printed = false;

    while (ros::ok()) {
        ros::Duration elapsed_time = ros::Time::now() - start_time;

        if (elapsed_time < ros::Duration(3.0)) {
            fake_controller.drone_cmd_reset();
            fake_controller.vehicle_cmd_reset();
        }
        else if (elapsed_time >= ros::Duration(3.0) && elapsed_time < ros::Duration(30.0)) {
            drone_state_msg.state = 1;
            drone_state_pub.publish(drone_state_msg);
            if (!stage_1_printed) {
                std::cout << GREEN << "阶段1: " << YELLOW << "无人机起飞..." << RESET << std::endl << std::endl;
                stage_1_printed = true;
            }
        }
        else if (elapsed_time >= ros::Duration(30.0) && elapsed_time < ros::Duration(40.0)) {
            drone_state_msg.state = 5;
            drone_state_pub.publish(drone_state_msg);
            fake_controller.drone_cmd_reset();
            fake_controller.vehicle_cmd_reset();
            if (!stage_2_printed) {
                std::cout << GREEN << "阶段2: " << YELLOW << "准备RC控制" << std::endl
                        << "    请打开遥控器RC控制开关!" << RESET << std::endl << std::endl;
                stage_2_printed = true;
            }
        }
        else if (elapsed_time >= ros::Duration(40.0) && elapsed_time < ros::Duration(50.0)) {
            drone_state_msg.state = 5;
            drone_state_pub.publish(drone_state_msg);
            fake_controller.drone_cmd_line(drone_line_speed);       
            fake_controller.vehicle_cmd_line(vehicle_line_speed); 
            if (!stage_3_printed) {
                std::cout << GREEN << "阶段3: " << YELLOW << "执行动作" << std::endl
                        << CYAN << "    无人机直线飞行，无人车直线前进" << RESET << std::endl << std::endl;
                stage_3_printed = true;
            }
        }
        else if (elapsed_time >= ros::Duration(50.0) && elapsed_time < ros::Duration(53.0)) {
            drone_state_msg.state = 5;
            drone_state_pub.publish(drone_state_msg);
            fake_controller.drone_cmd_reset();
            fake_controller.vehicle_cmd_reset();
            if (!stage_4_printed) {
                std::cout << GREEN << "阶段4: " << YELLOW << "等待下一组指令..." << RESET << std::endl << std::endl;
                stage_4_printed = true;
            }
        }
        else if (elapsed_time >= ros::Duration(53.0) && elapsed_time < ros::Duration(63.0)) {
            drone_state_msg.state = 5;
            drone_state_pub.publish(drone_state_msg);
            fake_controller.vehicle_cmd_line(-vehicle_line_speed);       
            fake_controller.drone_cmd_circle(drone_line_speed, drone_angular_speed);
            if (!stage_5_printed) {
                std::cout << GREEN << "阶段5: " << YELLOW << "执行动作" << std::endl
                        << CYAN << "    无人机绕圆周飞行，无人车直线后退" << RESET << std::endl << std::endl;
                stage_5_printed = true;
            }
        }
        else if (elapsed_time >= ros::Duration(63.0) && elapsed_time < ros::Duration(70.0)) {
            drone_state_msg.state = 5;
            drone_state_pub.publish(drone_state_msg);
            fake_controller.drone_cmd_reset();
            fake_controller.vehicle_cmd_reset();
            if (!stage_6_printed) {
                std::cout << GREEN << "阶段6: " << YELLOW << "准备返航" << std::endl
                        << "    请将遥控器RC控制关闭!" << RESET << std::endl << std::endl;
                stage_6_printed = true;
            }
        }
        else if (elapsed_time >= ros::Duration(70.0)) {
            drone_state_msg.state = 3;
            drone_state_pub.publish(drone_state_msg);
            if (!stage_7_printed) {
                std::cout << GREEN << "阶段7: " << YELLOW << "正在返航" << RESET << std::endl << std::endl;
                stage_7_printed = true;
            }
        }

        ros::spinOnce(); 
        loop_rate.sleep(); 
    }

    return 0;
}
