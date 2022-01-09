# LiDAR to Rosbag

把本地的雷达点云转换为一个rosbag，并且可以自定义转换前与转换后点云的坐标系变换。

支持坐标系变换主要是为了方便把rosbag放入不同的系统中运行。比如 LOAM 系列的代码都是默认按照 x向前 y向左 z向上的顺序存储点云。但是在采集过程中可能不是这个顺序，可能是 x向右 y向前 z向上。因此需要进行一个转换才可以。

## 使用方法

```bash
source ./devel/setup.bash
rosrun rosrun lidar2rosbag lidar2rosbag path output.bag xyz -yxz
```

`path` 是雷达点云存储的目录，默认以 `.pcd`格式为结尾，如果需要可以在程序中改成别的格式

`output.bag` 是保存的rosbag包的地方

`xyz` 可选参数，输入的雷达坐标顺序，按照 **右-前-上** 的顺序进行。比如 *xyz* 就代表 x向右 y向前 z向上

`-yxz` 可选参数，输出的雷达坐标顺序，按照 **右-前-上** 的顺序进行。比如 *-yxz* 就代表 y向左 x向前 z向上