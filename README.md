# Agilex-robots-tracker

## 安装依赖换环境
1.QSQP安装

```shell
git clone --recursive https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make
sudo make install
```

2.QSQP-Eigen安装

```shell
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build 
cd build
cmake ..
make
sudo make install
```

3.casadi求解器安装
```shell
pip install casadi
```

## 文件说明
1.启动hunter跟踪一个路点的仿真

`roslaunch tracker_demo single_hunter_track.launch`

2.启动scout跟踪一个路点的仿真

`roslaunch tracker_demo single_scout_track.launch`

3.启动hunter跟踪scout的仿真

`roslaunch tracker_demo hunter_track_scout.launch`

4.启动scout跟踪hunter的仿真

`roslaunch tracker_demo scout_track_hunter.launch`

注意:启动前需要对相关配置文件进行修改,[here](tracker_demo/README.md)