"""Planar inverse kinematics derived from this robot's URDF joint axes.

This maps body twists to ideal wheel targets, not to motor currents.
Units: metres, radians, seconds. Ground contact and no slip are assumptions.
"""
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class Module:
    name: str
    x: float
    y: float
    radius: float
    rolling_angle: float
    max_speed: float


@dataclass(frozen=True)
class Target:
    steering: float
    velocity: float


def load_modules(urdf: str | Path) -> tuple[Module, ...]:
    robot = ET.parse(urdf).getroot()
    modules = []
    for name in ('lh1', 'rh1', 'lh2', 'rh2'):
        steer = robot.find(f"joint[@name='{name}_steer_joint']")
        wheel = robot.find(f"joint[@name='{name}_wheel_joint']")
        x, y, _ = map(float, steer.find('origin').get('xyz').split())
        ax, ay, az = map(float, wheel.find('axis').get('xyz').split())
        if abs(az) > 1e-9 or abs(hypot(ax, ay) - 1) > 1e-9:
            raise ValueError('Wheel axis must be a horizontal unit vector')
        limit = steer.find('limit')
        if float(limit.get('lower')) > -pi/2 or float(limit.get('upper')) < pi/2:
            raise ValueError('This solver requires at least +/- pi/2 steering')
        cylinder = robot.find(f"link[@name='{name}_wheel']/collision/geometry/cylinder")
        radius = float(cylinder.get('radius'))
        speed = float(wheel.find('limit').get('velocity'))
        if radius <= 0 or speed <= 0:
            raise ValueError('Radius and wheel speed must be positive')
        # Positive rolling direction is axis cross +Z, not always body +X.
        modules.append(Module(name, x, y, radius, atan2(-ax, ay), speed))
    return tuple(modules)


def solve(modules: tuple[Module, ...], vx: float, vy: float, wz: float,
          previous: dict[str, float] | None = None) -> dict[str, Target]:
    if not all(isfinite(value) for value in (vx, vy, wz)):
        raise ValueError('Twist must be finite')
    previous = previous or {}
    targets = {}
    for m in modules:
        ux, uy = vx - wz * m.y, vy + wz * m.x
        speed = hypot(ux, uy)
        if speed < 1e-10:
            targets[m.name] = Target(previous.get(m.name, 0.0), 0.0)
            continue
        angle = (atan2(uy, ux) - m.rolling_angle + pi) % (2*pi) - pi
        sign = 1.0
        if angle > pi/2:
            angle -= pi
            sign = -1.0
        elif angle < -pi/2:
            angle += pi
            sign = -1.0
        targets[m.name] = Target(angle, sign * speed / m.radius)
    # One scale preserves the commanded body twist direction under saturation.
    scale = max([1.0] + [abs(targets[m.name].velocity)/m.max_speed for m in modules])
    return {name: Target(t.steering, t.velocity/scale) for name, t in targets.items()}


def wheel_vector(module: Module, target: Target) -> tuple[float, float]:
    a = module.rolling_angle + target.steering
    return (module.radius * target.velocity * cos(a),
            module.radius * target.velocity * sin(a))
