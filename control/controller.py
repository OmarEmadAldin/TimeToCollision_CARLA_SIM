import carla
import config

class AEBController:
    def __init__(self, traffic_manager=None, target_speed_diff=None,
                 warning_speed_diff=None):
        
        self.traffic_manager = traffic_manager
        self.normal_speed_diff = (
            target_speed_diff if target_speed_diff is not None else 20.0
        )
        self.warning_speed_diff = (
            warning_speed_diff if warning_speed_diff is not None else 60.0
        )
        self._autopilot_overridden = False

    def decide(self, tracked_detections):

        ttcs = [d["ttc"] for d in tracked_detections if d["ttc"] is not None]
        min_ttc = min(ttcs) if ttcs else float("inf")

        brake = min_ttc <= config.TTC_BRAKE_THRESHOLD
        warning = (not brake) and (min_ttc <= config.TTC_WARNING_THRESHOLD)

        return {
            "brake": brake,
            "warning": warning,
            "min_ttc": min_ttc,
        }

    def apply_control(self, vehicle, decision):

        if decision["brake"]:
            # Take manual control: full emergency brake, steer straight.
            if vehicle.get_control().throttle != 0.0 or not self._autopilot_overridden:
                vehicle.set_autopilot(False)
            self._autopilot_overridden = True

            control = carla.VehicleControl(
                throttle=0.0,
                brake=config.BRAKE_INTENSITY,
                steer=0.0,
                hand_brake=False,
            )
            vehicle.apply_control(control)
            return control

        # Not braking this tick: make sure autopilot is back in control.
        if self._autopilot_overridden:
            if self.traffic_manager is not None:
                vehicle.set_autopilot(True, self.traffic_manager.get_port())
            else:
                vehicle.set_autopilot(True)
            self._autopilot_overridden = False

        if self.traffic_manager is not None:
            if decision["warning"]:
                self.traffic_manager.vehicle_percentage_speed_difference(
                    vehicle, self.warning_speed_diff
                )
            else:
                self.traffic_manager.vehicle_percentage_speed_difference(
                    vehicle, self.normal_speed_diff
                )

        # Autopilot is driving; report back whatever control it applied
        # this tick so the HUD reflects the real throttle/brake values.
        return vehicle.get_control()

    @staticmethod
    def _get_speed_kmh(vehicle):
        velocity = vehicle.get_velocity()
        speed_ms = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
        return speed_ms * 3.6