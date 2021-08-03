# Slam-Project-Of-MyOwn
手写2D激光slam框架，基于图优化，scan to map 和回环检测

## 1. 开发环境
操作系统:
```shell
ubuntu16.04
```
第三方库:
```shell
Opencv3.4.11
Eigen2.0
```
请确保计算机正常安装以上两个库。

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
![image map_test](https://github.com/softdream/Slam-Project-Of-MyOwn/blob/master/doc/map.png)<br/>
--------------------------------------------------------------------------------------------------<br/>
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
Go into the build folder and execute the command
```shell
cd ./build
cmake ..
```
or
```shell
cd ./build
cmake -DLOGON=1 ..
```
to view the log.
Then execuate:
```shell
make
```
After that you can see two file in folder: build/lib
```shell
libslam.a libslam.so
```
You can use the lib file in your own project now.

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
