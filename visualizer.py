import cv2
import config


def draw_detections(frame, tracked_detections, ttc_estimator):

    for det in tracked_detections:
        x1, y1, x2, y2 = det["bbox"]
        distance = det.get("distance_smoothed")
        ttc = det.get("ttc", float("inf"))
        track_id = det.get("track_id", -1)

        risk = ttc_estimator.classify_ttc(ttc)
        color = {
            "safe": config.COLOR_SAFE,
            "warning": config.COLOR_WARNING,
            "danger": config.COLOR_DANGER,
        }[risk]

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, config.LINE_THICKNESS)

        dist_str = f"{distance:.1f}m" if distance is not None else "N/A"
        ttc_str = "inf" if ttc == float("inf") else f"{ttc:.1f}s"

        label = f"ID {track_id} | {dist_str} | TTC {ttc_str}"

        label_size, _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, config.FONT_SCALE, 1
        )
        label_w, label_h = label_size

        cv2.rectangle(
            frame,
            (x1, y1 - label_h - 8),
            (x1 + label_w + 4, y1),
            color,
            -1,
        )
        cv2.putText(
            frame,
            label,
            (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            config.FONT_SCALE,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    return frame


def draw_hud(frame, decision, ego_speed_kmh, control):
    
    if decision["brake"]:
        status_text = "AEB: EMERGENCY BRAKE"
        status_color = config.COLOR_DANGER
    elif decision["warning"]:
        status_text = "AEB: WARNING"
        status_color = config.COLOR_WARNING
    else:
        status_text = "AEB: SAFE"
        status_color = config.COLOR_SAFE

    min_ttc = decision["min_ttc"]
    min_ttc_str = "inf" if min_ttc == float("inf") else f"{min_ttc:.2f}s"

    lines = [
        status_text,
        f"Min TTC: {min_ttc_str}",
        f"Ego speed: {ego_speed_kmh:.1f} km/h",
        f"Throttle: {control.throttle:.2f}  Brake: {control.brake:.2f}",
    ]

    y = 25
    for i, line in enumerate(lines):
        color = status_color if i == 0 else (255, 255, 255)
        cv2.putText(
            frame,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            config.FONT_SCALE,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            config.FONT_SCALE,
            color,
            1,
            cv2.LINE_AA,
        )
        y += 25

    return frame
