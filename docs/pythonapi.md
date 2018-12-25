# CARLA PythonAPI 教程

## Intro
三个概念：
1. Actor: 任何一个仿真中起作用的东西，并且可以被移动。例如：车辆，行人，传感器；
2. Buleprint：在你把Actor放到仿真中之前，你需要制定它的属性，这个就是buleprints其作用的地方；
3. World：表示当前载入的地图，并且将一个blueprint转换成仿真中“活的”ACtor；同时，它也提供访问road map和改变天气的功能。

## Connecting and retrieving the world
1.创建一个client
`client = carla.Client('localhost', 2000)`
建议设置一个timeout时间：
`client.set_timeout(10.0) #seconds`
2.通过client获得world 
`world = client.get_world()`
后面基本就用不到client了。

## Blueprints
1.获得blueprint library
`buleprint_library = world.get_buleprint_library()`
2.blueprint library通过find(filter)找到指定ID的blueprints，进而可以修改它的属性
`collision_sensor_bp = blueprint_library.find('sensor.other.collision')`
或者创建一辆车
`my_vehicle = random.choice(blueprint_library.filter('vehicle.bmw.*'))`

## 放置actors
```
transform = transform(Location(x=230, y=195, z=40), Rotation(yaw=180)
actor = world.spawn_actor(my_vehicle, transform)
```
有时候，放置会出现错误，可能是指定的地方有静态物体或者当时有其他车辆，也可以用`try_spawn_actor()`函数。world提供了函数可以查看推荐的地点，避免了类似的放置的问题：
`spawn_points = world.get_map().get_spawn_points()`
最后，`spawn_actor()`函数还提供了另外一个选项，即是否attach到其他某个actor上，一般这种都是用在传感器上:
`camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle)`

### 操作actors

移动actor到到另一个位置：
```
location = actor.get_location()
location.z += 10.0
actor.set_location(location)
print(actor.get_acceleration())
print(actor.get_velocity())

```
当不需要这个actor，就把它destroy掉，因为Python脚本结束不会自动清除actor。
`actor.destroy()`

### Vehicles
Vehicle是一种特殊的actor，需要用`vehicle.apply_control(carla.VehicleControl(throttle=1.0,steer=-1.0))`来控制。完整的VehicleControl如下：
```
carla.VehicleControl(
    throttle = 0.0
    steer = 0.0
    brake = 0.0
    hand_brake = False
    reverse = False
    manual_gear_shift
    gear = 0
)

```
也可以设置自动模式：
`vehicle.set_autopilot(True)`

Vechiles也可以有一个3D的bounding box:
```
box = vehicle.bounding_box
print(box.location)
print(box.extent)
```

### 传感器Sensors
Camera传感器示例：
```
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle)
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame_number))
```
### 其他actors
可以用`actot_list = world.get_actors()`

### 地图和waypoints
地图是兼容OpenDrive格式的
1.获得地图
`map = world.get_map()`
可以让地图提供放置vehicle的位置：`map.get_spawn_points()`
2.获得**Waypoints**
我们可以让地图给我提供距离我们现在位置的最近的**waypoints**：`waypoint=map.get_waypoint(vehicle.get_location())`
waypoint在可行驶的路上，它根据道路方向确定朝向。

**Waypoints**也可以查询下一堆**waypoints**，它返回的是一系列在当前**waypoints**下可能车可能走的点。看个例子：
```
# Retrieve the closest waypoint.
waypoint = map.get_waypoint(vehicle.get_location())

# Disable physics, in this example we're just teleporting the vehicle.
vehicle.set_simulate_physics(False)

while True:
    # Find next waypoint 2 meters ahead.
    waypoint = random.choice(waypoint.next(2.0))
    # Teleport the vehicle.
    vehicle.set_transform(waypoint.transform)
```
map对象还可以提供在一个大致距离生成waypoints的功能：
`waypoint_list=map.generate_waytpoints(2.0)`
也可以获得用于导航的道路拓扑图：
`waypoint_tuple_list = map.get_topology()`


