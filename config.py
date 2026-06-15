# ----------------------------------------------------------------------
# CARLA connection
# ----------------------------------------------------------------------
CARLA_HOST = "localhost"
CARLA_PORT = 2000
CARLA_TIMEOUT = 10.0
CARLA_TOWN = "Town03"  # Town03/Town05 have good pedestrian crossings

# Synchronous simulation settings
FIXED_DELTA_SECONDS = 0.05  # 20 FPS sim step -> matches camera sensor_tick

# ----------------------------------------------------------------------
# Sensors
# ----------------------------------------------------------------------
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
CAMERA_FOV = 90

# Camera mounted on the hood, looking forward
CAMERA_TRANSFORM = {
    "x": 1.5,
    "y": 0.0,
    "z": 1.6,
    "pitch": 0.0,
    "yaw": 0.0,
    "roll": 0.0,
}

# CARLA depth camera encodes distance using:
#   normalized = (R + G*256 + B*256*256) / (256^3 - 1)
#   depth_meters = normalized * 1000.0   (far plane = 1000m)
DEPTH_FAR_PLANE_METERS = 1000.0

# ----------------------------------------------------------------------
# Pedestrians
# ----------------------------------------------------------------------
NUM_PEDESTRIANS = 250
PEDESTRIAN_SPEED_RANGE = (1.0, 2.0)  # m/s walking speed range

YOLO_MODEL_PATH = "yolov8n.pt"  # COCO-pretrained, class 0 = "person"
YOLO_CONF_THRESHOLD = 0.4
YOLO_PERSON_CLASS_ID = 0
YOLO_DEVICE = "cuda"  # falls back to "cpu" automatically if unavailable

# ----------------------------------------------------------------------
# Distance estimation
# ----------------------------------------------------------------------
# How to sample the depth map within a bounding box.
# "center"       -> single pixel at bbox center
# "median_box"   -> median over a small patch around the center (robust to
#                    noise / partial occlusion at edges)
DEPTH_SAMPLE_MODE = "median_box"
DEPTH_SAMPLE_PATCH_FRACTION = 0.2  # patch size = 20% of bbox width/height

# ----------------------------------------------------------------------
# TTC computation
# ----------------------------------------------------------------------
# Exponential moving average smoothing factor for distance (0 < alpha <= 1).
# Lower alpha = smoother but more lag.
DISTANCE_EMA_ALPHA = 0.4

# Minimum closing speed (m/s) before we consider the pedestrian "approaching".
# Below this, TTC is reported as infinity (no collision risk from this metric).
MIN_CLOSING_SPEED = 0.05

# ----------------------------------------------------------------------
# AEB (Autonomous Emergency Braking) decision logic
# ----------------------------------------------------------------------
# Euro NCAP AEB pedestrian tests typically evaluate response within ~1.5-2.0s TTC.
TTC_WARNING_THRESHOLD = 2.0   # seconds -> visual/audible warning
TTC_BRAKE_THRESHOLD = 1.2     # seconds -> apply emergency brake
BRAKE_INTENSITY = 1.0         # 0.0-1.0

# Ego vehicle target cruising speed (km/h) before AEB intervenes
EGO_TARGET_SPEED_KMH = 30.0

# ----------------------------------------------------------------------
# Visualization / output
# ----------------------------------------------------------------------
WINDOW_NAME = "Pedestrian Collision Warning"
COLOR_SAFE = (0, 255, 0)        # green  (TTC > warning threshold)
COLOR_WARNING = (0, 255, 255)   # yellow (warning <= TTC < brake)
COLOR_DANGER = (0, 0, 255)      # red    (TTC <= brake threshold)
FONT_SCALE = 0.6
LINE_THICKNESS = 2

# ----------------------------------------------------------------------
# Recording
# ----------------------------------------------------------------------
OUTPUT_DIR = "output"
OUTPUT_VIDEO_FILENAME = "pedestrian_ttc_demo.mp4"
OUTPUT_VIDEO_FPS = 20