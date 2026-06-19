"""Fixed-waypoint navigation for the simple map.

This is a plain Python module first. It can be wrapped as a ROS node later.
"""

from config import (
    DEFAULT_FORWARD_SPEED,
    DEFAULT_TURN_SPEED,
    FORWARD_DISTANCE_TOLERANCE_M,
    MAX_FORWARD_SEGMENT_M,
    REALIGN_AFTER_SEGMENT,
    ROUTES,
    SEGMENT_PAUSE_S,
)
import time
from robot_driver import RobotDriver


class NavigationNode:
    def __init__(self, driver=None, routes=None):
        self.driver = driver or RobotDriver()
        self.routes = routes or ROUTES

    def move_between(self, start, goal, speed=DEFAULT_FORWARD_SPEED):
        key = (start, goal)
        if key not in self.routes:
            known = ", ".join(f"{src}->{dst}" for src, dst in self.routes)
            raise ValueError(f"No route for {start}->{goal}. Known routes: {known}")

        route = self.routes[key]
        print(f"[INFO] Route {start} -> {goal}: {route}")
        self.execute_route(route, speed=speed)
        print(f"[INFO] Arrived at {goal}")

    def execute_route(self, route, speed=DEFAULT_FORWARD_SPEED):
        for action, value in route:
            if action == "forward":
                progress = self.go_forward_segmented(value, speed=speed)
                if progress + FORWARD_DISTANCE_TOLERANCE_M < value:
                    raise RuntimeError(
                        f"Forward move failed: target={value:.2f}m, "
                        f"progress={progress:.2f}m"
                    )
            elif action == "turn_left":
                self.driver.turn_by(value, angular_speed=DEFAULT_TURN_SPEED)
            elif action == "turn_right":
                self.driver.turn_by(-value, angular_speed=DEFAULT_TURN_SPEED)
            else:
                raise ValueError(f"Unknown navigation action: {action}")

    def go_forward_segmented(self, distance_m, speed=DEFAULT_FORWARD_SPEED):
        remaining = distance_m
        total_progress = 0.0
        target_yaw = self.driver.get_yaw() if not self.driver.dry_run else 0.0
        while remaining > FORWARD_DISTANCE_TOLERANCE_M:
            segment = min(remaining, MAX_FORWARD_SEGMENT_M)
            progress = self.driver.go_forward(segment, speed=speed)
            total_progress += progress
            if progress + FORWARD_DISTANCE_TOLERANCE_M < segment:
                return total_progress
            remaining -= progress
            if REALIGN_AFTER_SEGMENT:
                self.driver.realign_to_yaw(target_yaw)
            if remaining > FORWARD_DISTANCE_TOLERANCE_M:
                time.sleep(SEGMENT_PAUSE_S)
        return total_progress

    def move_flow(self, waypoints, speed=DEFAULT_FORWARD_SPEED):
        for start, goal in zip(waypoints, waypoints[1:]):
            self.move_between(start, goal, speed=speed)
