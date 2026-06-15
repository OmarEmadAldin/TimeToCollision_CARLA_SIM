import numpy as np
import config


def estimate_distance(depth_map, bbox, mode=None):
    mode = mode or config.DEPTH_SAMPLE_MODE

    x1, y1, x2, y2 = bbox
    h, w = depth_map.shape

    # Clamp to image bounds
    x1 = max(0, min(x1, w - 1))
    x2 = max(0, min(x2, w - 1))
    y1 = max(0, min(y1, h - 1))
    y2 = max(0, min(y2, h - 1))

    if x2 <= x1 or y2 <= y1:
        return None

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    if mode == "center":
        return float(depth_map[cy, cx])

    if mode == "median_box":
        box_w = x2 - x1
        box_h = y2 - y1

        patch_w = max(1, int(box_w * config.DEPTH_SAMPLE_PATCH_FRACTION))
        patch_h = max(1, int(box_h * config.DEPTH_SAMPLE_PATCH_FRACTION))

        px1 = max(0, cx - patch_w // 2)
        px2 = min(w, cx + patch_w // 2 + 1)
        py1 = max(0, cy - patch_h // 2)
        py2 = min(h, cy + patch_h // 2 + 1)

        patch = depth_map[py1:py2, px1:px2]
        if patch.size == 0:
            return float(depth_map[cy, cx])

        return float(np.median(patch))

    raise ValueError(f"Unknown DEPTH_SAMPLE_MODE: {mode}")


class DistanceTracker:
    
    def __init__(self, alpha=None, max_match_distance_px=80):
        self.alpha = alpha or config.DISTANCE_EMA_ALPHA
        self.max_match_distance_px = max_match_distance_px
        self._tracks = {}  # track_id -> {"center": (x,y), "distance_ema": float}
        self._next_id = 0

    def update(self, detections_with_distance):
       
        results = []
        used_track_ids = set()

        for det in detections_with_distance:
            bbox = det["bbox"]
            cx = (bbox[0] + bbox[2]) / 2.0
            cy = (bbox[1] + bbox[3]) / 2.0

            track_id = self._associate((cx, cy), used_track_ids)

            raw_distance = det["distance"]
            if raw_distance is None:
                smoothed = self._tracks.get(track_id, {}).get("distance_ema")
            else:
                prev = self._tracks.get(track_id, {}).get("distance_ema")
                if prev is None:
                    smoothed = raw_distance
                else:
                    smoothed = self.alpha * raw_distance + (1 - self.alpha) * prev

            self._tracks[track_id] = {"center": (cx, cy), "distance_ema": smoothed}
            used_track_ids.add(track_id)

            det_out = dict(det)
            det_out["track_id"] = track_id
            det_out["distance_smoothed"] = smoothed
            results.append(det_out)

        return results

    def _associate(self, center, used_track_ids):
        best_id = None
        best_dist = self.max_match_distance_px

        for track_id, track in self._tracks.items():
            if track_id in used_track_ids:
                continue
            tx, ty = track["center"]
            d = ((center[0] - tx) ** 2 + (center[1] - ty) ** 2) ** 0.5
            if d < best_dist:
                best_dist = d
                best_id = track_id

        if best_id is not None:
            return best_id

        new_id = self._next_id
        self._next_id += 1
        return new_id
