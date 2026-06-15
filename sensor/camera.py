import queue
import numpy as np
from config import *

class SensorSync:

    def __init__(self, rgb_camera, depth_camera):
        self.rgb_camera = rgb_camera
        self.depth_camera = depth_camera

        self._rgb_queue = queue.Queue()
        self._depth_queue = queue.Queue()

        self.rgb_camera.listen(self._rgb_queue.put)
        self.depth_camera.listen(self._depth_queue.put)

    def get_frames(self, frame_id, timeout=2.0):
        """
        Block until the RGB and depth images matching `frame_id`
        (the simulation frame number returned by world.tick()) are
        retrieved. Returns (rgb_array_bgr, depth_array_meters).
        """
        rgb_image = self._retrieve(self._rgb_queue, frame_id, timeout)
        depth_image = self._retrieve(self._depth_queue, frame_id, timeout)

        rgb_array = carla_image_to_bgr_array(rgb_image)
        depth_array = carla_depth_to_meters(depth_image)

        return rgb_array, depth_array

    @staticmethod
    def _retrieve(sensor_queue, frame_id, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == frame_id:
                return data

    def stop(self):
        self.rgb_camera.stop()
        self.depth_camera.stop()


def carla_image_to_bgr_array(carla_image):
    """
    Convert a carla.Image (sensor.camera.rgb) into a standard
    OpenCV-friendly BGR numpy array of shape (H, W, 3).
    """
    raw = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
    array = raw.reshape((carla_image.height, carla_image.width, 4))  # BGRA
    bgr = array[:, :, :3].copy()
    return bgr


def carla_depth_to_meters(carla_depth_image):

    raw = np.frombuffer(carla_depth_image.raw_data, dtype=np.uint8)
    array = raw.reshape(
        (carla_depth_image.height, carla_depth_image.width, 4)
    )  # BGRA

    # array channel order is B, G, R, A
    b = array[:, :, 0].astype(np.float32)
    g = array[:, :, 1].astype(np.float32)
    r = array[:, :, 2].astype(np.float32)

    normalized = (r + g * 256.0 + b * 256.0 * 256.0) / (256.0 ** 3 - 1.0)
    depth_meters = normalized * DEPTH_FAR_PLANE_METERS

    return depth_meters