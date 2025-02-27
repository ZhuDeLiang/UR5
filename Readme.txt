依赖项：
1 ur_rtde
安装方式1
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
安装方式2
官方源代码安装

##ur-rtde 依赖boost pybind 通过pip install/ apt install 安装

2 Eigen3
安装方式1
sudo apt install libeigen3-dev
安装方式2
源码安装

#####源码安装需手动在cmakelist中添加include lib src