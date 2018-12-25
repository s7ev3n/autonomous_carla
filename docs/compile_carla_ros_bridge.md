# CARLA编译与官方ros_bridge

>注意：目前官方的ros_bridge只适用在0.9.2版本和之后的版本。参见：[ros-bridge/issue14](https://github.com/carla-simulator/ros-bridge/issues/14#issuecomment-449347438).

## 编译CARLA

> 简单记录一下编译CARLA的过程。因为ros_bridge文章中并没有说明0.9.1版本不能与其工作，所以为了解决问题将CARLA的中间版本编译了，见[issue](https://github.com/carla-simulator/carla/issues/1068#issuecomment-449372914)。之后ros_bridge更新了文档，并且发布了正式的0.9.2，预计之后不需要自己编译CARLA. 

1. [阅读官方编译文档](https://carla.readthedocs.io/en/latest/how_to_build_on_linux/)

2. 注册Unreal

3. Clone下来carla[源码](https://github.com/carla-simulator/carla)

4. `./Update.sh`，这一步会要求下载源码版本的Content文件(Google drive)，在国内基本没有办法下载，可以手动下载，然后解压在指定位置。

5. 编译CARLA，在carla源码文件夹执行下面命令，主要有：

   ```shell
   make launch #编译carla，并启动Unreal Engine's Editor
   make PythonAPI #编译PythonAPI，ros_bridge需要用到
   make package #编译carla，并打包成可执行文件，和官方release的一样。这一步时间很长，大约得有半小时。
   ```

6. [另一篇参考](https://www.codetd.com/article/2757436)

### 注意的地方：

编译CARLA需要用到`libpng16-dev`，即支持保存png格式的图片，但是ros中的`cv_bridge`需要的`opencv`依赖`libpng12-dev`，两者有一些冲突，参见[issue901](https://github.com/carla-simulator/carla/issues/901#issuecomment-449059971)和[issue924](https://github.com/carla-simulator/carla/pull/924)，解决的这种办法是编译时去掉对png图片的支持，在`setup.py` 文件中设置`-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=false`，然后进行上面按步骤编译。

However，这个问题可能有可能得到了解决，见[issue](https://github.com/carla-simulator/ros-bridge/issues/14#issuecomment-449700827) ，然而并没有尝试。你可以自己试一下。

## ros_bridge

按照项目主页的步骤来就行：

1. 建立catkin workspace

2. 安装PythonAPI

3. 运行CARLA server : `./CarlaUE4.sh  -carla-server -windowed -ResX=320 -ResY=240`

4. 启动ros_bridge: `roslaunch carla_ros_bridge client.launch`  [目前我这里rviz无法启动，一直卡在初始化界面，原因未知。]

5. `python manual_control.py`

6. 发指令

   ```
   rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
     jerk: 0.0}" -r 10
   ```

7. 可以看到车子跑直线跑起来，至此ros_bridge可以和CARLA连接起来，你也可以查看各个topic的信息

##### 小Tip：

`./CarlaUE4.sh  -carla-server -windowed -ResX=320 -ResY=240`默认是启动server模式，即使去掉`-carla-server`也是启动server模式，server模式会以第三视角随意移动查看环境，但是当有client和它连接后（默认在2000端口），这个窗口依然没有变化，但实际上已经连接上。



