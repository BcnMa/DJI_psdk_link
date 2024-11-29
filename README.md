# DJI Matrice 350 psdk 通讯及测试包

## 快速使用
 1. 打开psdk通讯
```shell
roslaunch psdk psdk.launch
```

 2. 发布运动指令
```shell
roslaunch fake_controller fake_rc_pub.launch
```



## 更新日志
### 2024.11.29
 - 撰写README
 - 该项目包移植到air_ground项目下
 
 
 
### 2024.11.28
 - 完成fake_controller包，提供rc控制及位置控制的话题
 

 
### 2024.11.23
 - 完成psdk包在ROS系统下的移植
 - 完善psdk通讯接口
