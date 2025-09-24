import math
from mathutils import Vector

def distance_point_to_line(start: Vector, end: Vector, point: Vector) -> float:
    A = (point - start).cross(point - end).length
    L = (end - start).length
    return A / L

def wrap_angle(angle_rad: float) -> int:
    "Convert radians to 0–255 XML value."
    angle_rad = angle_rad % (2 * math.pi)
    return round(angle_rad / (2 * math.pi) * 255) & 0xFF