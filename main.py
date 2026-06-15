
import sys
import time
import cv2
import config
from world_spawn.spawn_manager import setup_world, teardown_world
from sensor.camera import SensorSync
from detector.detector import PedestrianDetector
from detector.distance_estimation import estimate_distance, DistanceTracker
from control.ttc import TTCEstimator
from control.controller import AEBController
from visualizer import *
from save_vid import *

# --- video recording settings ---
SAVE_VIDEO = True
OUTPUT_VIDEO_FILENAME = None  # None -> uses config.OUTPUT_VIDEO_FILENAME


def main():
    print("Connecting to CARLA and spawning actors...")
    actors = setup_world()
    world = actors["world"]
    ego_vehicle = actors["ego_vehicle"]
    rgb_camera = actors["rgb_camera"]
    depth_camera = actors["depth_camera"]
    sensor_sync = SensorSync(rgb_camera, depth_camera)
    detector = PedestrianDetector()
    distance_tracker = DistanceTracker()
    ttc_estimator = TTCEstimator()
    aeb_controller = AEBController()

    # --- video recorder setup ---
    output_path = default_output_path(OUTPUT_VIDEO_FILENAME) if SAVE_VIDEO else None
    recorder = VideoRecorder(output_path=output_path)

    prev_time = None
    try:
        while True:
            frame_id = world.tick()
            rgb_frame, depth_map = sensor_sync.get_frames(frame_id)
            now = time.time()
            dt = (now - prev_time) if prev_time is not None else config.FIXED_DELTA_SECONDS
            prev_time = now
            detections = detector.detect(rgb_frame)
            for det in detections:
                det["distance"] = estimate_distance(depth_map, det["bbox"])
            tracked = distance_tracker.update(detections)
            tracked = ttc_estimator.update(tracked, dt)
            decision = aeb_controller.decide(tracked)
            control = aeb_controller.apply_control(ego_vehicle, decision)

            # --- visualization ---
            ego_speed_kmh = aeb_controller._get_speed_kmh(ego_vehicle)
            display_frame = rgb_frame.copy()
            draw_detections(display_frame, tracked, ttc_estimator)
            draw_hud(display_frame, decision, ego_speed_kmh, control)

            # --- save video (no-op if SAVE_VIDEO is False) ---
            recorder.write(display_frame)

            cv2.imshow(config.WINDOW_NAME, display_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Cleaning up...")
        sensor_sync.stop()
        recorder.release()
        cv2.destroyAllWindows()
        teardown_world(actors)
        print("Done.")


if __name__ == "__main__":
    sys.exit(main())