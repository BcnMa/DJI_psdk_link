[toc]
# PSDK 空地协同通讯包
## 用法
需要采用ROS通讯，设置psdk.launch的ctrl_mode为1。

port_num默认为0，指向/dev/ttyUSB0，目前只有/dev/ttyUSB1和/dev/ttyUSB2这三种端口，通过改变port_num实现一一对应。

debug_mode默认为true，false时不打印。

```shell
roslaunch psdk pskd.launch

# 单独启动节点，不推荐使用
rosrun psdk psdk_node
```

如果出现uart open error报错，需要考虑：
 1. 是否没有插入通讯模块（数传）
 2. 需要sudo chmod 777 /dev/ttyUSB*给端口权限

## 更新日志
### 2024.11.28
 - 更新pub_pose()函数，无人机端NX提供的自身位置为经纬度

### 2024.11.23
 - 将代码移植到ROS系统
 - 设置两种模式，由ros param的ctrl_mode控制，为0时是阻塞输入指令，为1时是ros通讯指令