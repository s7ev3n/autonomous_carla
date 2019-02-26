
import carla
import random
import rospy
import time


def main():
    try:
        actor_list = []
        # Create client and get the world
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        bluelib = world.get_blueprint_library()

        # Create a vehicle and spawn into the world
        bp = random.choice(bluelib.filter('vehicle'))
        transform = random.choice(world.get_map().get_spawn_points())
        print(transform)
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
        #set autopilot
        vehicle.set_autopilot(True)

        #add a lidar
        lidar_bp = bluelib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '10000')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('points_per_second', '600000')
        lidar_bp.set_attribute('rotation_frequency', '10.0')
        lidar_bp.set_attribute('upper_fov', '30.0')
        lidar_bp.set_attribute('lower_fov','-30.0')
        lidar_transform = carla.Transform(carla.Location(z=2.))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        actor_list.append(lidar)
   
        while True:
            print(vehicle.get_location()) # Location(x=-74.2983, y=-2.73689, z=-0.0425265)
            print(vehicle.get_acceleration()) # Vector3D(x=0.000635669, y=0.240702, z=-0.000339426)
            # print(vehicle.get_rotation()) # do not have this func
            print(vehicle.get_transform()) # Transform(Location(x=-74.2983, y=-2.73689, z=-0.0425265), Rotation(pitch=0.00756785, yaw=-90.0157, roll=-0.0502625))
            print(vehicle.get_transform().rotation) # Get rotation from Transform
            time.sleep(0.1)

        time.sleep(5)
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')

if __name__ == '__main__':

    main()
