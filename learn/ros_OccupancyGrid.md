# OccupancyGrid Map 占据删格地图
> 在学习Autoware的时![地图](https://github.com/CPFL/Autoware/tree/master/ros/src/computing/perception/semantics/packages/object_map)对其中坐标问题不是很理解：1.OccupancyGrid Map中的`origin`是什么；2.为什么要有offset？？

## 解释下这个`origin`
下面是`nav_msgs/MapMetaData.msg`这种msg格式的具体
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
注意：分辨率的单位是 m/cell。
这么看可能也不是很明白，


REF：
1.https://blog.csdn.net/qq_16775293/article/details/84645528
2.https://blog.csdn.net/sru_alo/article/details/84770855
