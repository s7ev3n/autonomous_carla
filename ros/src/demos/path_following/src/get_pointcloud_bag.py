
import carla
import random

# Create client and get the world
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()
bluelib = world.get_blueprint_library()

#Create a vehicle and spawn into the world
bp = random.choice(bluelib.filter('vehicle'))
transform = random.choice(world.get_map().get_spawn_points())
vehicle = world.spawn_actor(bp, transform)
#set autopilot
vehicle.set_autopilot(True)

#add a lidar
lidar_bp = bluelib.find('sensor.lidar.ray_cast')
lidar_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

location = vehicle.get_location()
print('Location', location)


