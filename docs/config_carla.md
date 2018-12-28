# Carla启动方式

1. CARLA启动方式是`./CarlaUE4.sh -xx`，执行后会出现一个Carla模拟器的画面，你可以用WASD和鼠标随意看看Carla世界。这种启动方式默认是以server的启动方式启动，特点是等待一个client在端口（默认是localhost:2000）连接，client连接后加入一个actor这个画面（可能）也没变化，这一点还有待进一步验证。这种启动方式，需要自己写一个client.py的文件，其中得到Blueprints，并对车辆和传感器进行配置，这些是写在代码中完成的。

2. 配置文件启动
Carla也可以用配置文件的方式启动: `./CarlaUE4.sh -carla-settings=Path/to/CarlaSetting.ini`
这种启动方式的优点是，把blueprints中用代码配置的部分省去。具体的`CarlaSetting.ini`的写法，可以参见carla源码中Docs文件下的`Example.CarlaSetting.ini`文件。

