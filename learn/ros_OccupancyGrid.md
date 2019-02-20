# OccupancyGrid Map 占据删格地图
> 记录学习Autoware的时![占据删格地图](https://github.com/CPFL/Autoware/tree/master/ros/src/computing/perception/semantics/packages/object_map)

## OccupancyGrid Map 占据删格地图
什么是占据删格地图呢？它将空间网格化，然后每个格子表示是否被占据的概率，0表示未被占据，100表示占据，-1表示不确定。
参看这篇知乎文章：https://zhuanlan.zhihu.com/p/21738718
那么它和Costmap（代价地图）有什么区别呢？我们来看下面的解释：
nav_msgs/OccupancyGrid is a ROS message. This is a map that can be passed around between tools, namely gmapping/karto, amcl, and move_base.

costmap2d is a ROS package, which offers a costmap generation tool, which in fact is really 2.5D. That is, it does raytracing from sensor data into a 3d-voxel map, but it outputs a 2d costmap. It is mainly used via a C++ api, not the ROS message api. When used over ROS messages, it posts nav_msgs/GridCells on a variety of topics corresponding to obstacles, inflated obstacles, and unknown space.

Now, in the context of global planner, all planners use the C++ costmap2d interface. And thus, the global costmap2d is typically initialized from some map, which is typically received over a latched topic using nav_msgs/OccupancyGrid. The costmap2d then can do things needed for planning, like inflating obstacles or adding sensor data to the costmap if desired/configured as such.
上面的意思表示，OccupancyGrid是ROS中的一种消息格式，而costmap代价地图主要用于路径规划，用于描述环境中的障碍物和自由空间。通常也用栅格地图表示，常见的整体代价地图和分层代价地图。有一个包costmap2d，是一个包用于生成costmap。

我们暂且认为Autoware中的OccupancyGrid图和Costmap是一样的。

## OccupancyGrid消息格式
我们先来看OccupancyGrid，它是ROS中的一种消息格式在`nav_msgs/OccupancyGrid`下面（其实还有一种叫做`nav_msgs/GridCells`消息，不同的是，它可以表示成是三维的。）这个消息中最重要的是`nav_msgs/MapMetaData.msg`这种msg格式，具体如下：
```
# This hold basic information about the characterists of the OccupancyGrid
# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
```
说明一下：
resolution是分辨率的单位是 m/cell；
width和height是，我们这个删格地图的长宽各有多少个格子，可以理解成像素；
origin指的是这个删格地图原点在真实世界坐标系的坐标点，需要注意一点：世界坐标系的x轴和y轴是和grid地图同方向的，即体现在`og.header = input_cloud.header`。

## obj_map中的origin设置
激光雷达得到的一帧点云数据是以自身为中心的圆形，坐标系的原点是激光雷达中心。我们如果把origin的设置position和orientation(w不为0)设置为0（默认也是0），那意味着占据删格地图画出了第一象限。但是一般情况下，我们想画出以车为中心（或者有一点偏离的，比如前方多看点）的删格图，这时候我们需要把origin设置成第三象限的点，例如Autoware中：
```
og->info.origin.position.x = (-1) * g_map_width / 2. + g_offset_x;
og->info.origin.position.y = (-1) * g_map_height / 2. + g_offset_y);
```
这个时候就表示把车辆中心，即世界坐标系的原点放在了og map的中心啦。


## `cost_map[index] += 15`
首先我们要知道index是怎么算的：
```
int grid_x = (p.x + map_center_x) / g_resolution;
int grid_y = (p.y + map_center_y) / g_resolution;
int index = g_cell_width * grid_y + grid_x;

```
我们遍历一帧点云所有得到的点，把每个点调整到以origin为中心的坐标系（`p.x+map_center_x`），除以分辨率然后取整就可以得到grid_x啦！由于costmap其实就是一个一维数组来存储的，但是它有二维关系，因为它可以转化成二维的图啊。所以`index = g_cell_width * grid_y + grid_x`。

那么，`cost_map[index] += 15`是什么呢，这个15是怎么来的呢？
因为很多点会四舍五入到一个小格子里面，如果一个小格子里面有好几个点，那这表明很有可能这个小格子是占用的，因此，累加15，这个15纯粹是人为设置的参数。如果这个小格子里面的参数超出了100，那么得给他限制到100，因为它不可能超过100的。


知道这些，差不多就可以啦。








REF：
1.https://blog.csdn.net/qq_16775293/article/details/84645528
2.https://blog.csdn.net/sru_alo/article/details/84770855
