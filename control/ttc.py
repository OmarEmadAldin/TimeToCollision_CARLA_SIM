'''
Time-To-Collision computation. TTC = distance / closing_speed

closing_speed is the rate at which the gap between the ego vehicle and
the pedestrian is shrinking, in m/s. We derive it by differentiating the
(smoothed) distance signal over time:

    closing_speed = (distance_prev - distance_curr) / dt

If closing_speed <= MIN_CLOSING_SPEED (i.e. the pedestrian is stationary
relative to the ego vehicle, or moving away), TTC is reported as
float('inf') -- there's no collision risk from this metric.
'''

import config


class TTCEstimator:

    def __init__(self, min_closing_speed=None):
        self.min_closing_speed = min_closing_speed or config.MIN_CLOSING_SPEED
        self._prev_distances = {}  # track_id -> distance (meters)

    def update(self, tracked_detections, dt):
        results = []

        for det in tracked_detections:
            track_id = det["track_id"]
            distance = det["distance_smoothed"]

            closing_speed = 0.0
            ttc = float("inf")

            prev_distance = self._prev_distances.get(track_id)

            if distance is not None and prev_distance is not None and dt > 0:
                closing_speed = (prev_distance - distance) / dt

                if closing_speed > self.min_closing_speed:
                    ttc = distance / closing_speed
                else:
                    ttc = float("inf")

            if distance is not None:
                self._prev_distances[track_id] = distance

            det_out = dict(det)
            det_out["closing_speed"] = closing_speed
            det_out["ttc"] = ttc
            results.append(det_out)

        return results

    def classify_ttc(self, ttc):

        if ttc <= config.TTC_BRAKE_THRESHOLD:
            return "danger"
        if ttc <= config.TTC_WARNING_THRESHOLD:
            return "warning"
        return "safe"
