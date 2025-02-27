依赖项：
1 ur_rtde
安装方式1
sudo附加功能重复ppa：sdurobotics/ur-rtde
sudo apt-get更新
sudo apt install librtde librtde-dev
安装方式2
官方源代码安装

## ur-rtde依赖提升pybind通过pip安装/ apt安装安装

2 eigen3
安装方式1
sudo apt install libeigen3-dev
安装方式2
源码安装

#####源码安装需手动在cmakelist中添加包括lib src

使用说明：
测试为测试文件，演示了如何调用UR5的控制命令函数
