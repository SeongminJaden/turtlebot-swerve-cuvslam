"""Prototype for an arm-mounted camera task; not wired into MazeEnv yet.

Tracking validity must come from a timestamped tracker adapter, never merely
from camera alignment or the presence of an odometry message. Limits below
are experiment starting points, not validated hardware safety settings.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ViewGateConfig:
    enter_angle: float = math.radians(15)
    exit_angle: float = math.radians(25)
    direction_change: float = math.radians(20)
    dwell_s: float = 0.5
    max_sample_age_s: float = 0.15
    max_step_s: float = 0.15
    stopped_speed: float = 0.02
    stopped_yaw_rate: float = 0.03
    settled_arm_speed: float = 0.05


class ViewGate:
    """Fail-closed command gate with stop-before-look and continuous verification.

    STOP -> LOOK -> VERIFY -> DRIVE. A new travel direction, a stale tracker,
    an infeasible view or invalid timing revokes DRIVE immediately. A zero
    command requests braking; actual stopping distance still needs validation.
    Caller must apply the result to *all* base commands, including yaw hold.
    """
    def __init__(self, config=ViewGateConfig()):
        self.config = config
        self.reset()

    def reset(self):
        self.state = "STOP"
        self.good_time = 0.0
        self.direction = None

    def step(self, *, dt, desired_heading, alignment_error, tracking_ok,
             sample_age, reachable, base_speed, base_yaw_rate, arm_speed):
        c = self.config
        numbers = (dt, desired_heading, alignment_error, sample_age,
                   base_speed, base_yaw_rate, arm_speed)
        if (not all(math.isfinite(x) for x in numbers)
                or not 0 < dt <= c.max_step_s
                or sample_age < 0 or not reachable):
            self.reset()
            return False
        # A stale tracker blocks driving, but must not prevent a stopped arm
        # from looking for a view in which tracking can initialize/recover.
        tracking_ok = tracking_ok and sample_age <= c.max_sample_age_s
        change = (self.direction is None or abs(math.atan2(
            math.sin(desired_heading-self.direction),
            math.cos(desired_heading-self.direction))) > c.direction_change)
        if change:
            self.reset()
            self.direction = desired_heading
        if self.state == "DRIVE":
            if (tracking_ok and abs(alignment_error) <= c.exit_angle
                    and abs(arm_speed) <= c.settled_arm_speed):
                return True
            self.state, self.good_time = "STOP", 0.0
            return False
        stopped = (abs(base_speed) <= c.stopped_speed and
                   abs(base_yaw_rate) <= c.stopped_yaw_rate)
        if not stopped:
            self.state, self.good_time = "STOP", 0.0
            return False
        self.state = "LOOK"
        good = (tracking_ok and abs(alignment_error) <= c.enter_angle and
                abs(arm_speed) <= c.settled_arm_speed)
        if good:
            self.state = "VERIFY"
            self.good_time += dt
            if self.good_time + 1e-9 >= c.dwell_s:
                self.state = "DRIVE"
                return True
        else:
            self.good_time = 0.0
        return False


def reward_terms(*, dt, progress, angle_error, previous_angle_error,
                 drive_allowed, base_speed, arm_speed_squared,
                 ready_event=False, collision_event=False, success_event=False):
    """Unweighted terms; no sustained alignment bonus that rewards staying still.

    progress is signed path progress in metres, not velocity. ready_event is
    supplied once per navigation segment; never repeatedly pay for re-entering
    VERIFY. Apply separate weights and log every term before PPO experiments.
    """
    return {
        "progress": progress if drive_allowed else 0.0,
        "look_progress": max(-1., min(1., (abs(previous_angle_error)-abs(angle_error))/math.pi)),
        "ready": float(ready_event),
        "blind_motion": -abs(base_speed)*dt if not drive_allowed else 0.0,
        "arm_effort_proxy": -arm_speed_squared*dt,
        "time": -dt,
        "collision": -float(collision_event),
        "success": float(success_event),
    }
