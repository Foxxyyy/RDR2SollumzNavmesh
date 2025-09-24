from collections import defaultdict
from math import acos, degrees, radians
import bpy
import re
import bmesh
from bpy.types import (
    Object,
    Mesh,
    MeshPolygon,
)
from collections.abc import Iterator

from ..tools.utils import get_min_vector_list
from ..cwxml.navmesh import NavPolygon, Navmesh
from ..sollumz_properties import SollumType
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

_NAVMESH_MAP_NAME_REGEX = re.compile(r"^.*\[(\d+)\]\[(\d+)\].*$")

NAVMESH_GRID_SIZE = 215
NAVMESH_GRID_CELL_SIZE = 150.0
NAVMESH_GRID_BOUNDS_MIN = Vector((-15900.0, -15900.0, 0.0))
NAVMESH_GRID_BOUNDS_MIN.freeze()
NAVMESH_GRID_BOUNDS_MAX = \
NAVMESH_GRID_BOUNDS_MIN + Vector((NAVMESH_GRID_SIZE, NAVMESH_GRID_SIZE, 0.0)) * NAVMESH_GRID_CELL_SIZE
NAVMESH_GRID_BOUNDS_MAX.freeze()

NAVMESH_SECTORS_PER_GRID_CELL = 3
NAVMESH_STANDALONE_CELL_INDEX = 10000
NAVMESH_ADJACENCY_INDEX_NONE = 0x3FFF
NAVMESH_POLY_SMALL_MAX_AREA = 2.0
NAVMESH_POLY_LARGE_MIN_AREA = 40.0

global global_vertices, vertex_list
global_vertices = {}
vertex_list = []

def navmesh_is_valid(obj: Object | None) -> bool:
    """Gets whether the object is a navmesh."""
    return obj is not None and obj.sollum_type == SollumType.NAVMESH and obj.type in {"MESH", "EMPTY"}


def navmesh_is_map(obj: Object) -> bool:
    """Gets whether the object is a navmesh placed in the map grid. We identify map navmeshes by their name, checking if
    it contains a '[123][456]' identifier.
    """
    return navmesh_is_valid(obj) and _NAVMESH_MAP_NAME_REGEX.match(obj.name)


def navmesh_is_standalone(obj: Object):
    """Gets whether the object is a standalone navmesh, not in the map grid (i.e. vehicle navmeshes)."""
    return navmesh_is_valid(obj) and not navmesh_is_map(obj)


def navmesh_get_grid_cell(obj: Object) -> tuple[int, int]:
    """Gets the cell coordinates of a map navmesh."""
    if not navmesh_is_valid(obj):
        return -1, -1

    match = _NAVMESH_MAP_NAME_REGEX.match(obj.name)
    if not match:
        return -1, -1

    x = int(match.group(1))
    y = int(match.group(2))
    return x // NAVMESH_SECTORS_PER_GRID_CELL, y // NAVMESH_SECTORS_PER_GRID_CELL


def navmesh_grid_get_cell_filename(x: int, y: int) -> str:
    sx = x * NAVMESH_SECTORS_PER_GRID_CELL
    sy = y * NAVMESH_SECTORS_PER_GRID_CELL
    return f"navmesh[{sx}][{sy}]"


def navmesh_grid_get_cell_bounds(x: int, y: int) -> tuple[Vector, Vector]:
    cell_min = NAVMESH_GRID_BOUNDS_MIN + Vector((x, y, 0.0)) * NAVMESH_GRID_CELL_SIZE
    cell_max = cell_min + Vector((NAVMESH_GRID_CELL_SIZE, NAVMESH_GRID_CELL_SIZE, 0.0))
    return cell_min, cell_max


def navmesh_grid_get_cell_index(x: int, y: int) -> int:
    return y * NAVMESH_GRID_SIZE + x


def navmesh_grid_is_cell_valid(x: int, y: int) -> bool:
    return 0 <= x < NAVMESH_GRID_SIZE and 0 <= y < NAVMESH_GRID_SIZE


def navmesh_grid_get_cell_neighbors(x: int, y: int) -> Iterator[tuple[int, int]]:
    if navmesh_grid_is_cell_valid(x - 1, y):
        yield x - 1, y
    if navmesh_grid_is_cell_valid(x, y - 1):
        yield x, y - 1
    if navmesh_grid_is_cell_valid(x + 1, y):
        yield x + 1, y
    if navmesh_grid_is_cell_valid(x, y + 1):
        yield x, y + 1


def navmesh_compute_cell(obj: Object) -> tuple[int, int]:
    bbmin = get_min_vector_list(obj.bound_box)
    center = Vector(((bbmin.x + obj.bound_box[6][0]) / 2, (bbmin.y + obj.bound_box[6][1]) / 2, 0.0))

    # Offset relative to grid origin
    offset = center - NAVMESH_GRID_BOUNDS_MIN

    # Divide by cell size
    x = int(offset.x // NAVMESH_GRID_CELL_SIZE)
    y = int(offset.y // NAVMESH_GRID_CELL_SIZE)

    return x, y


def get_adj_area_ids(navmesh_obj: Object) -> list[int]:
        """Return the AdjAreaIDs for a navmesh object"""
        area_ids = [navmesh_obj.get("adj_area_ids")[0], 65535]
        
        if navmesh_is_map(navmesh_obj):
            cell_x, cell_y = navmesh_get_grid_cell(navmesh_obj)
            for nx, ny in navmesh_grid_get_cell_neighbors(cell_x, cell_y):
                neighbor_area_id = navmesh_grid_get_cell_index(nx, ny)
                if neighbor_area_id not in area_ids:
                    area_ids.append(neighbor_area_id)
        return area_ids


def _loop_to_half_edge(mesh: Mesh, loop_idx: int) -> tuple[int, int]:
    loop = mesh.loops[loop_idx]
    edge_verts = mesh.edges[loop.edge_index].vertices
    v0, v1 = edge_verts
    if v0 != loop.vertex_index:
        v1, v0 = edge_verts
    assert v0 == loop.vertex_index, \
        f"Degenerate mesh, failed to get half-edge from loop: {v0=}, {v1=}, {loop_idx=}, {loop.vertex_index=}, {loop.edge_index=}"
    return v0, v1


def navmesh_compute_edge_adjacency(mesh: Mesh) -> tuple[dict[tuple[int, int], int], dict[tuple[int, int], int]]:
    half_edge_to_lhs_poly = {}
    half_edge_to_rhs_poly = {}

    for poly in mesh.polygons:
        for loop_idx in poly.loop_indices:
            v0, v1 = _loop_to_half_edge(mesh, loop_idx)

            assert (v0, v1) not in half_edge_to_lhs_poly, \
                f"Degenerate mesh, multiple LHS polygons on half-edge ({v0}, {v1}): {half_edge_to_lhs_poly[(v0, v1)]} and {poly.index}"

            half_edge_to_lhs_poly[(v0, v1)] = poly.index

    for poly in mesh.polygons:
        for loop_idx in poly.loop_indices:
            v0, v1 = _loop_to_half_edge(mesh, loop_idx)

            # The RHS poly of this half-edge is the LHS poly of the half-edge going on the opposite direction
            rhs_poly_idx = half_edge_to_lhs_poly.get((v1, v0), None)
            if rhs_poly_idx is not None:
                assert (v0, v1) not in half_edge_to_rhs_poly, \
                    f"Degenerate mesh, multiple RHS polygons on half-edge ({v0}, {v1}): {half_edge_to_rhs_poly[(v0, v1)]} and {rhs_poly_idx}"
                half_edge_to_rhs_poly[(v0, v1)] = rhs_poly_idx

    return half_edge_to_lhs_poly, half_edge_to_rhs_poly


def navmesh_poly_get_adjacent_polys_local(mesh: Mesh, poly_idx: int, edge_adjacendy: tuple[dict[tuple[int, int], int], dict[tuple[int, int], int]]):
    _, half_edge_to_rhs_poly = edge_adjacendy

    adjacent_polys = []
    poly = mesh.polygons[poly_idx]
    for loop_idx in poly.loop_indices:
        v0, v1 = _loop_to_half_edge(mesh, loop_idx)

        adjacent_poly = half_edge_to_rhs_poly.get((v0, v1), None)
        if adjacent_poly is None:
            adjacent_poly = NAVMESH_ADJACENCY_INDEX_NONE

        adjacent_polys.append(adjacent_poly)

    return adjacent_polys


def navmesh_poly_update_flags(mesh: Mesh, poly_idx: int):
    from .navmesh_attributes import mesh_get_navmesh_poly_attributes, mesh_set_navmesh_poly_attributes

    poly = mesh.polygons[poly_idx]
    poly_attrs = mesh_get_navmesh_poly_attributes(mesh, poly_idx)

    area = poly.area
    poly_attrs.is_small = area < NAVMESH_POLY_SMALL_MAX_AREA
    poly_attrs.is_large = area > NAVMESH_POLY_LARGE_MIN_AREA

    mesh_set_navmesh_poly_attributes(mesh, poly_idx, poly_attrs)


def is_walkable(verts):
    """Check if polygon slope is shallow enough to be walkable"""
    if len(verts) < 3:
        return False
    v1, v2, v3 = verts[:3]
    normal = (v2 - v1).cross(v3 - v1)
    if normal.length == 0:
        return False
    normal.normalize()
    dot = max(min(normal.dot(Vector((0, 0, 1))), 1.0), -1.0)
    angle = degrees(acos(dot))

    return angle <= 55.0


def flatten_stairs(verts, threshold=0.1):
    min_z = min(v.z for v in verts)
    max_z = max(v.z for v in verts)
    if max_z - min_z < threshold:
        avg_z = sum(v.z for v in verts) / len(verts)
        return [Vector((v.x, v.y, avg_z)) for v in verts]
    return verts


def polygon_area(verts):
    if len(verts) < 3: return 0
    area = 0
    for i in range(len(verts)):
        v0 = verts[i]
        v1 = verts[(i+1) % len(verts)]
        area += v0.x * v1.y - v1.x * v0.y
    return abs(area) * 0.5


def snap(value, step):
    return round(value / step) * step


def quantize(v, step=0.05, step_z=0.2):
    return (snap(v.x, step), snap(v.y, step), snap(v.z, step_z))


def gather_bounds(obj, bounds_list):
        if obj.type == 'MESH':
            bounds_list.extend([obj.matrix_world @ Vector(corner) for corner in obj.bound_box])
        for child in obj.children:
            gather_bounds(child, bounds_list)
            

def intersect(p1, p2, t):
    return Vector((
        p1.x + (p2.x - p1.x) * t,
        p1.y + (p2.y - p1.y) * t,
        p1.z + (p2.z - p1.z) * t
    ))


def clip_polygon(polygon, xmin, xmax, ymin, ymax):
    """Clip polygon to an axis-aligned bounding box"""
    def clip_edge(verts, inside_fn, edge_fn):
        out = []
        if not verts:
            return out
        prev, prev_inside = verts[-1], inside_fn(verts[-1])
        for curr in verts:
            curr_inside = inside_fn(curr)
            if curr_inside:
                if not prev_inside:
                    t = edge_fn(prev, curr)
                    out.append(intersect(prev, curr, t))
                out.append(curr)
            elif prev_inside:
                t = edge_fn(prev, curr)
                out.append(intersect(prev, curr, t))
            prev, prev_inside = curr, curr_inside
        return out

    verts = polygon
    verts = clip_edge(verts, lambda v: v.x >= xmin, lambda p1,p2:(xmin-p1.x)/(p2.x-p1.x))
    verts = clip_edge(verts, lambda v: v.x <= xmax, lambda p1,p2:(xmax-p1.x)/(p2.x-p1.x))
    verts = clip_edge(verts, lambda v: v.y >= ymin, lambda p1,p2:(ymin-p1.y)/(p2.y-p1.y))
    verts = clip_edge(verts, lambda v: v.y <= ymax, lambda p1,p2:(ymax-p1.y)/(p2.y-p1.y))
    return verts


def filter_topmost(polygons):
    # Build BVH from all triangles
    all_verts = []
    all_tris = []
    vidx = 0
    for poly in polygons:
        verts = poly.vertices
        idxs = list(range(vidx, vidx+3))
        all_verts.extend(verts)
        all_tris.append(idxs)
        vidx += 3
    bvh = BVHTree.FromPolygons(all_verts, all_tris)

    filtered = []
    for poly in polygons:
        cx = sum(v.x for v in poly.vertices) / 3.0
        cy = sum(v.y for v in poly.vertices) / 3.0
        cz = sum(v.z for v in poly.vertices) / 3.0
        origin = Vector((cx, cy, cz + 50.0))  # start ray well above
        direction = Vector((0, 0, -1))

        hit, normal, index, dist = bvh.ray_cast(origin, direction)
        if hit is not None:
            # Keep only if this polygon is the hit one
            tri_z = cz
            hit_z = hit.z
            if abs(tri_z - hit_z) < 0.1:  # tolerance
                filtered.append(poly)

    return filtered


def filter_stacked_polys(polygons, z_threshold=0.05, xy_prec=3):
    """Remove duplicate polys stacked on top of each other."""
    buckets = {}
    filtered = []

    def xy_key(poly):
        # Sort vertices by XY so order doesn't matter
        verts2d = sorted([(round(v.x, xy_prec), round(v.y, xy_prec)) for v in poly.vertices])
        return tuple(verts2d)

    for poly in polygons:
        key = xy_key(poly)
        cz = sum(v.z for v in poly.vertices) / len(poly.vertices)

        if key not in buckets:
            buckets[key] = (cz, poly)
        else:
            top_z, top_poly = buckets[key]
            if abs(cz - top_z) <= z_threshold:
                continue
            elif cz > top_z:
                # Replace with higher poly
                buckets[key] = (cz, poly)

    filtered = [p for (_, p) in buckets.values()]
    return filtered


def weld_vertices(polygons, step_xy=0.05, step_z=0.2, eps=0.15):
    """Merge vertices with quantization + distance-based cleanup."""

    # Quantization weld
    vertex_map = {}
    welded_polys = []

    for poly in polygons:
        welded_verts = []
        for v in poly.vertices:
            key = (snap(v.x, step_xy), snap(v.y, step_xy), snap(v.z, step_z))
            if key not in vertex_map:
                vertex_map[key] = Vector(key)
            welded_verts.append(vertex_map[key])

        new_poly = NavPolygon()
        new_poly.vertices = welded_verts
        new_poly.edges = poly.edges
        new_poly.flags = poly.flags
        new_poly.centroid_x = sum(v.x for v in welded_verts) / len(welded_verts)
        new_poly.centroid_y = sum(v.y for v in welded_verts) / len(welded_verts)
        welded_polys.append(new_poly)

    # Distant cleanup
    grid = {}
    cell_size = eps

    def grid_key(v):
        return (int(v.x // cell_size), int(v.y // cell_size), int(v.z // cell_size))

    merged = []
    final_map = {}

    for poly in welded_polys:
        for v in poly.vertices:
            gk = grid_key(v)
            found = None

            # Only check neighbors in adjacent cells
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        cell = grid.get((gk[0]+dx, gk[1]+dy, gk[2]+dz))
                        if cell:
                            for m in cell:
                                if (v - m).length <= eps:
                                    found = m
                                    break
                        if found:
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                found = v
                merged.append(v)
                grid.setdefault(gk, []).append(v)
            final_map[(v.x, v.y, v.z)] = found

    # Rebuild merged verts
    final_polys = []
    for poly in welded_polys:
        new_poly = NavPolygon()
        new_poly.vertices = [final_map[(v.x, v.y, v.z)] for v in poly.vertices]
        new_poly.edges = poly.edges
        new_poly.flags = poly.flags
        new_poly.centroid_x = sum(v.x for v in new_poly.vertices) / len(new_poly.vertices)
        new_poly.centroid_y = sum(v.y for v in new_poly.vertices) / len(new_poly.vertices)
        final_polys.append(new_poly)

    return final_polys


def collision_to_navmesh(objs, cell_min_x, cell_max_x, cell_min_y, cell_max_y): 
    polygons = []
    
    def add_triangle(tri, is_terrain):
        nav_poly = NavPolygon()
        nav_poly.vertices = tri
        nav_poly.edges = [(0, 0, 0, 0) for _ in range(3)]
        nav_poly.flags = 0
        nav_poly.centroid_x = sum(v.x for v in tri) / 3.0
        nav_poly.centroid_y = sum(v.y for v in tri) / 3.0
        nav_poly.is_terrain_poly = is_terrain
        polygons.append(nav_poly)

    def export_bound(obj, is_terrain = False):
        if obj.sollum_type == SollumType.BOUND_POLY_TRIANGLE and obj.type == 'MESH':        
            mesh = obj.data
            materials = mesh.materials
            for poly in mesh.polygons:
                verts = [obj.matrix_world @ mesh.vertices[i].co for i in poly.vertices]
                if materials and poly.material_index < len(materials):
                    mat = materials[poly.material_index]
                    flags = mat.collision_flags

                    if is_terrain:
                        if not is_walkable(verts):
                            continue
                    else:
                        if polygon_area(verts) < 0.05:
                            continue

                    clipped = clip_polygon(verts, cell_min_x, cell_max_x, cell_min_y, cell_max_y)       
                    if len(clipped) >= 3:
                        for i in range(1, len(clipped) - 1): # Triangulate if > 3 verts
                            tri = [clipped[0], clipped[i], clipped[i + 1]]
                            add_triangle(tri, is_terrain)
            return
        else:
            if obj.sollum_type == SollumType.BOUND_COMPOSITE:
                is_terrain = len(obj.children) > 1
            for child in obj.children: # Composite + GeometryBVH
                export_bound(child, is_terrain)

    for obj in objs:
        export_bound(obj)

    if not polygons:
        return None
    
    # Compute bounds of the generated polygons
    min_x = min(v.x for p in polygons for v in p.vertices)
    max_x = max(v.x for p in polygons for v in p.vertices)
    min_y = min(v.y for p in polygons for v in p.vertices)
    max_y = max(v.y for p in polygons for v in p.vertices)

    size_x = round(max_x - min_x, 3)
    size_y = round(max_y - min_y, 3)

    # Skip if not exactly 150 × 150 (with tolerance)
    if abs(size_x - 150.0) > 0.01 or abs(size_y - 150.0) > 0.01:
        return None

    terrain_polys = [tp for tp in polygons if tp.is_terrain_poly]
    terrain_polys = filter_topmost(terrain_polys)
    terrain_polys = weld_vertices(terrain_polys)
    terrain_polys = filter_stacked_polys(terrain_polys)

    building_polys = [tp for tp in polygons if not tp.is_terrain_poly]
    building_polys = weld_vertices(building_polys)
    building_polys = filter_stacked_polys(building_polys)

    final_polys = terrain_polys + building_polys

    navmesh_xml = Navmesh()
    navmesh_xml.polygons = final_polys
    navmesh_xml.adj_area_ids = " ".join(map(str, [0] * len(terrain_polys)))
    navmesh_xml.build_id = 0

    return navmesh_xml