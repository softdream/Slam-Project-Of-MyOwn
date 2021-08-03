# Slam-Project-Of-MyOwn
手写2D激光slam框架，基于图优化，scan to map 和回环检测

## 1. 开发环境
操作系统:
```shell
ubuntu16.04或以上版本
```
第三方库:
```shell
Opencv3.4.11或以上版本
Eigen2.0或以上版本
```
请确保计算机正常安装以上两个库。

编译环境:
```shell
cmake3.5.1或以上版本
g++5.1或以上版本
```

## 2. 使用方法
### 2.1 生成静态链接库和动态链接库
在工程根目录下新建文件夹build
```shell
mkdir build 
```
进入build文件夹并执行cmake
```shell
cd ./build
cmake ..
```
如果想查看log，执行以下命令
```shell
cd ./build
cmake -DLOGON=1 ..
```
接着执行make命令开始编译
```shell
make
```
编译成功后可以在build/lib目录看到生成了.a文件和.so文件
```shell
libslam.a libslam.so
```
可直接将链接库用于您的工程项目当中。

### 2.2 使用仿真例程
进入test目录并新建build目录
```shell
cd ./test
mkdir build
```
进入build目录并执行cmake命令
```shell
cd ./build
cmake ..
```
接着执行make命令进行编译
```shell
make
```
编译成功后可以在build/bin目录下看到可执行文件
```shell
gridMapBaseTest  icpTest  scanMatchTest  slamSimulation
```
执行slamSimulation命令可进行建图仿真
```shell
./slamSimulation
```
运行结果如下：
![image map_test](https://github.com/softdream/Slam-Project-Of-MyOwn/blob/master/doc/map.png)<br>


## 3. 代码内容简介
### 3.1 数据预处理
  >>为了避免在Scan To Map的过程中位姿估计陷入局部最优值，现准备使用惯性测量单元(Inertial Measurement Unit, IMU)和里程计(Odometry)数据进行融合为Scan To Map位姿估计提供初值，在使用IMU之前要对其进行校准，标定出其零偏误差(Zero Offset Error)和标量误差(Scalar Error)。<br>
  >>零偏误差指传感器在零激励下的输出误差。<br>
  >>标量误差指传感器在一定激励下，其输出值与输入值的比值。<br>
  >>此外在机器人运动过程中，激光雷达测量会因为机器人的运动而产生畸变，需要对其去除。
  
#### 3.1.1 IMU中的陀螺仪校准
  >>陀螺仪的标量误差校准需要高精度的陀螺仪标定转台来完成，因此只对其零偏误差进行校准。<br>
  >>零偏误差的校准使用均值校准，即在IMU静止的情况下，采集一定量的陀螺仪数据求其均值即可。
  
#### 3.1.2 IMU中的加速度计校准(已完成)
  >>加速度的误差校准使用椭球拟合(Ellipsoid Fitting)方法。将零偏误差参数和标量误差参数作为待优化量，构建椭球方程，根据加速度计的测量值使用高斯牛顿法(Gauss Newton Method)求解此方程，得到误差参数。<br>
  >>椭球拟合方法也即现在常用的六面校准法。
  
#### 3.1.3 基于EKF的IMU和里程计数据融合(已完成)
  >>本方案中里程计(Odometry)使用的为霍尔编码器(Hall Encoder)。编码器可用来测量机器人行驶速度，对速度在时间上积分即可得到位移，因此里程计可用来估计机器人位姿变化情况。一般来说里程计和IMU随着时间的变化都会有累计误差，单独使用一个传感器误差会很大，因此可对其数据进行融合提高机器人位姿估计精度。<br>
  >>本项目采用的传感器数据融合方法为扩展卡尔曼滤波(Extended Kalman Filter, EKF)方法。<br>
  >>本项目中里程计和IMU融合的数据主要为Scan To Map方法提供初值，这一步是可选的，默认不加上。

#### 3.1.4 二维激光雷达运动学畸变去除(未完成)

### 3.2 SLAM前端
#### 3.2.1 概率占据栅格地图(已完成)
  >>一般在SLAM中地图的表示方法有很多种，有语义地图(The Semantic Map)，拓扑地图(The Topology Map)，占据栅格地图(Occupied Grid Map)。<br>
  >>本项目使用最常见的占据栅格地图，在占据栅格地图中，每一个栅格代表一个对应于现实世界中的在某个分辨率下的障碍物，使用占据率(Occupancy)来描述该栅格是障碍物的概率。<br>
  
#### 3.2.2 地图构建方法(已完成)
  >>占据栅格地图的构建有很多方法，本项目采用经典的激光雷达反演观测模型(Inverse Observation Model)来构建每一帧地图。
  
#### 3.2.3 Scan To Map方法(已完成)
  >>由于激光雷达测量值较为准确，因此几乎所有的SLAM方案中都会使用激光雷达测量值来对机器人进行位姿估计。位姿估计的过程称之为扫描匹配(Scan Matching)。<br>
  >>常见的扫描匹配方法有：<br>
  >>1. Scan To Scan的方法：迭代最近点算法(Iterative Closest Point, ICP)，只对当前帧的激光雷达扫描数据与上一帧的数据进行匹配，根据两帧数据间所有点与点之间的欧氏距离和来建立误差方程，使用高斯牛顿法或者奇异值分解(SVD)方法求解方程使得距离和最小，得到最佳的机器人位姿坐标变换。<br>
  >>2. Scan To Map的方法：和ICP方法不同，当前帧的激光雷达扫描数据与已建好的历史占据栅格地图进行匹配，根据激光雷达观测点在地图上占据栅格的概率值的和来建立误差方程，使用高斯牛顿法求解方程使得占据概率值的和最大，得到最佳的机器人位姿坐标变换。此外还使用了双线性插值方法来计算当前帧激光雷达数据在地图上对应点的占据概率，提高匹配精准度。<br>
  >>3. Scan To SubMap的方法：与Scan To Map方法不同，Scan To SubMap方法只将当前帧激光雷达数据与历史前几帧数据进行匹配，而不是所有历史帧进行匹配，谷歌开源的cartographer使用了此方法并使用了双三次插值方法提高精准度。
  >>4. CSM + 分支限界法：相关性扫描匹配(Correlation Scan Matching, CSM)方法即暴力匹配方法，在一个搜索窗口对所有激光点数据进行暴力匹配，为降低匹配时间，采用分支限界法对搜索窗口进行剪枝，减少匹配次数。谷歌开源的cartographer即采用此种方法来做回环检测。<br>
  >>5. NDT方法：正态分布变换(Normal Distribution Transformation)。<br>
 本项目采用Scan To Map的方法。参考了开源项目hector slam。
  
### 3.3 SLAM后端
#### 3.3.1 回环检测方法(待验证)
  >>回环检测(Loop Closure Detection)是用来检测机器人有没有在某一时刻运动到之前到过的点，如果是则构成一个回环，一个回环是一个强约束，通常与图优化方法相结合，用来对回环上的所有位姿估计进行优化，消除累计误差。<br>
  >>2d激光slam的回环检测方法有：<br>
  >>1. 基于特征的匹配<br>
  >>基于特征的匹配方法首先要求对激光雷达的观测数据进行特征提取(Feature Extraction)，由于是2维的建图，特征比较单一，完全没有视觉或者3d激光雷达的特征丰富，因此在特征提取这一阶段就很困难。一般有线特征，圆特征，直角特征等。<br>
  >>特征的匹配，待续。

  >>2. 基于点的匹配
  >>基于点的匹配方法和scan matching的方法相同，这里不再赘述。<br>
  >>本项目采用icp方法来进行回环检测，思路是对于每两帧雷达扫描数据，定义一个损失函数(Loss Function)，固定使用牛顿高斯法迭代100次，Loss小于阈值的认为匹配成功，视为检测到了回环。

#### 3.3.2 图优化方法(未完成)
  >>图优化的目的是从整体上对所有已估计到的并且在回环上的位姿进行优化，减小误差。图优化的思路是将位姿作为图(Graph)的顶点(Vertex),位姿间的坐标变换关系作为边(Edge), 顶点为优化变量，边为优化约束，构建出一个位姿图(Pose Graph)。有些时候需要将雷达观测点也作为图的顶点，观测点与机器人的位姿之间的关系也作为边构建出一个图，这种情况下称之为Bundle Adjustment(BA)问题，这里不做考虑。<br>
  >>图优化问题，实质上也是一个非线性最小二乘问题，只不过随着机器人的移动，位姿点会越来越多，此时构建的优化方程的规模会越来越大，使用高斯牛顿法无法直接对其求解。现有的解决方案是对求出的海森矩阵(Hessain Matrix)进行乔利斯基分解(Choleski Decomposition)，降低海森矩阵的规模，再使用高斯牛顿法进行求解即可。<br>
  >>本项目准备使用谷歌开源的g2o(General Graph Optimization)库来处理位姿图优化问题。

-------------------------------------------------------------------------------------------------------------------------<br/>
## 1.  Developement Environment
Operating System:
```shell
ubuntu16.04
```
Third-party Libraries:
```shell
Opencv3.4.11
Eigen2.0
```
Please make sure you have installed the above two libraries successfully.

## 2. Usage
### 2.1 Generate the static link library and dynamic link library
Create a new folder int the root directory of the project
```shell
mkdir build 
```
Go into the 'build' folder and execute the command
```shell
cd ./build
cmake ..
```
or
```shell
cd ./build
cmake -DLOGON=1 ..
```
to view the log.<br>
Then execuate:
```shell
make
```
After that you can see two files in folder: build/lib
```shell
libslam.a libslam.so
```
You can use the lib files in your own project now.

## 2.2 How to use the demo to mapping
First go into the 'test' folder and make a new folder named 'build':
```shell
cd ./test
mkdir build
```
Then execuate the commands:
```shell
cd ./build
cmake ..
make
```
After successful compilation, you can see the executable files in folder: 'build/bin'
```shell
gridMapBaseTest  icpTest  scanMatchTest  slamSimulation
```
Execuate the command to mapping:
```shell
./slamSimulation
```
