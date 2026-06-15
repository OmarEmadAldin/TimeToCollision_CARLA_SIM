import random
import carla
import config

def connect_to_carla():
    """Connect to the CARLA server and load the configured town."""
    client = carla.Client(config.CARLA_HOST, config.CARLA_PORT)
    client.set_timeout(config.CARLA_TIMEOUT)

    world = client.get_world()
    if world.get_map().name.split("/")[-1] != config.CARLA_TOWN:
        world = client.load_world(config.CARLA_TOWN)

    return client, world


def enable_synchronous_mode(world, client):
    
    settings = world.get_settings()
    original_settings = settings  # keep a copy to restore on cleanup

    settings.synchronous_mode = True
    settings.fixed_delta_seconds = config.FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    return original_settings, traffic_manager


def restore_world_settings(world, original_settings, traffic_manager=None):
    """Restore the world (and traffic manager) to its previous settings."""
    if traffic_manager is not None:
        try:
            traffic_manager.set_synchronous_mode(False)
        except RuntimeError:
            pass
    world.apply_settings(original_settings)


def spawn_ego_vehicle(world, traffic_manager):
    
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("model3")[0]  # Tesla Model 3

    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    vehicle.set_autopilot(True, traffic_manager.get_port())
    traffic_manager.vehicle_percentage_speed_difference(vehicle, 30.0)
    traffic_manager.auto_lane_change(vehicle, False)

    return vehicle


def spawn_cameras(world, vehicle):
    """
    Attach an RGB camera and a depth camera to the ego vehicle at the
    same transform, so their projections line up pixel-for-pixel.
    """
    blueprint_library = world.get_blueprint_library()

    transform = carla.Transform(
        carla.Location(
            x=config.CAMERA_TRANSFORM["x"],
            y=config.CAMERA_TRANSFORM["y"],
            z=config.CAMERA_TRANSFORM["z"],
        ),
        carla.Rotation(
            pitch=config.CAMERA_TRANSFORM["pitch"],
            yaw=config.CAMERA_TRANSFORM["yaw"],
            roll=config.CAMERA_TRANSFORM["roll"],
        ),
    )

    # --- RGB camera ---
    rgb_bp = blueprint_library.find("sensor.camera.rgb")
    rgb_bp.set_attribute("image_size_x", str(config.IMAGE_WIDTH))
    rgb_bp.set_attribute("image_size_y", str(config.IMAGE_HEIGHT))
    rgb_bp.set_attribute("fov", str(config.CAMERA_FOV))
    rgb_bp.set_attribute("sensor_tick", str(config.FIXED_DELTA_SECONDS))
    rgb_camera = world.spawn_actor(rgb_bp, transform, attach_to=vehicle)

    # --- Depth camera (same pose, same intrinsics) ---
    depth_bp = blueprint_library.find("sensor.camera.depth")
    depth_bp.set_attribute("image_size_x", str(config.IMAGE_WIDTH))
    depth_bp.set_attribute("image_size_y", str(config.IMAGE_HEIGHT))
    depth_bp.set_attribute("fov", str(config.CAMERA_FOV))
    depth_bp.set_attribute("sensor_tick", str(config.FIXED_DELTA_SECONDS))
    depth_camera = world.spawn_actor(depth_bp, transform, attach_to=vehicle)

    return rgb_camera, depth_camera


def spawn_pedestrians(client, world, num_pedestrians=None):

    if num_pedestrians is None:
        num_pedestrians = config.NUM_PEDESTRIANS

    blueprint_library = world.get_blueprint_library()
    walker_bps = blueprint_library.filter("walker.pedestrian.*")

    walkers = []
    controllers = []

    spawn_points = []
    for _ in range(num_pedestrians):
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            spawn_points.append(carla.Transform(loc))

    walker_controller_bp = blueprint_library.find("controller.ai.walker")

    for spawn_point in spawn_points:
        walker_bp = random.choice(walker_bps)
        # Pedestrians can be "invincible" by default; disable so AEB matters.
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "false")

        walker = world.try_spawn_actor(walker_bp, spawn_point)
        if walker is None:
            continue
        walkers.append(walker)

    world.tick()

    # Attach AI controllers
    for walker in walkers:
        controller = world.spawn_actor(
            walker_controller_bp, carla.Transform(), attach_to=walker
        )
        controllers.append(controller)

    world.tick()

    # Start walkers: walk to a random nearby point at a random speed
    for controller in controllers:
        controller.start()
        target_loc = world.get_random_location_from_navigation()
        controller.go_to_location(target_loc)
        speed = random.uniform(*config.PEDESTRIAN_SPEED_RANGE)
        controller.set_max_speed(speed)

    return list(zip(walkers, controllers))


def destroy_pedestrians(pedestrian_pairs):
    """Stop AI controllers and destroy both controller and walker actors."""
    for walker, controller in pedestrian_pairs:
        try:
            controller.stop()
        except RuntimeError:
            pass
        controller.destroy()
        walker.destroy()


def setup_world():
    """
    Convenience entrypoint used by main.py.
    Returns a dict with everything that needs to be ticked/cleaned up.
    """
    client, world = connect_to_carla()
    original_settings, traffic_manager = enable_synchronous_mode(world, client)

    ego_vehicle = spawn_ego_vehicle(world, traffic_manager)
    rgb_camera, depth_camera = spawn_cameras(world, ego_vehicle)
    pedestrian_pairs = spawn_pedestrians(client, world)

    return {
        "client": client,
        "world": world,
        "original_settings": original_settings,
        "traffic_manager": traffic_manager,
        "ego_vehicle": ego_vehicle,
        "rgb_camera": rgb_camera,
        "depth_camera": depth_camera,
        "pedestrian_pairs": pedestrian_pairs,
    }


def teardown_world(actors):
    """Destroy all spawned actors and restore world settings."""
    try:
        actors["rgb_camera"].destroy()
        actors["depth_camera"].destroy()
    except RuntimeError:
        pass

    destroy_pedestrians(actors["pedestrian_pairs"])

    try:
        actors["ego_vehicle"].set_autopilot(False)
        actors["ego_vehicle"].destroy()
    except RuntimeError:
        pass

    restore_world_settings(
        actors["world"], actors["original_settings"], actors.get("traffic_manager")
    )