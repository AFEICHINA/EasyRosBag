# EasyRosBag
```
x86_64: Test on Ubuntu18.04
arm64 : Test on Ubuntu21.04(raspberry Pi4)
```
## Introducton
ROS(Robot Operation System) is too fat!!!

We do not install ROS in our platform sometimes. In order to read Rosbag files on these platform, EasyRosBag is comming!

## Installation
```
git clone https://gitee.com/afeiii/EasyRosbag.git
mkdir build
cd build
cmake ..
make ..
```

## Dependency
1.Eigen=3.3.4

2.Pangolin

3.EMQX

## Run
1.Modify parameters.txt
```
cd rosbag/config
modify ip=your localhost, 
for example: board_ip=192.168.1.26
```
2.Run EMQX

EMQX is an MQTT broker, We use it to publish/subscribe rosbag messages. you can downloads it at
[EMQX link]("https://www.emqx.com/en/downloads"), please choose correct version. And start it 

```
cd emqx/bin
./emqx start
```


3.Run exe
```
cd ../bin 
./rosbag_play YourRosbag.bag
```

```
./rosbag_viz
```

if you meet error like this:
```
error while loading shared libraries: libmosquitto.so.1: cannot open shared object file: No such file or directory
```
you should 
```
export LD_LIBRARY_PATH=/YourProjectPath/EasyRosBag/tools/OpenSourceLib/x86_64/libmosquito_x86_64/lib:$LD_LIBRARY_PATH
```


