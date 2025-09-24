from collections import defaultdict
import bpy
import bmesh
from bpy.types import (
    Object
)
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import math

from .properties import NavEdgeFlags
from ..shared.math import wrap_angle
from typing import Optional
from ..cwxml.navmesh import (
    Navmesh,
    NavPoint,
    NavLink,
    NavPolygon,
)
from ..sollumz_properties import SollumType
from ..tools.utils import (
    get_max_vector_list,
    get_min_vector_list,
)
from ..tools.blenderhelper import (
    get_evaluated_obj,
)
from .navmesh import (
    get_adj_area_ids,
    navmesh_is_valid,
    navmesh_is_standalone,
    navmesh_is_map,
    navmesh_get_grid_cell,
    navmesh_grid_get_cell_bounds,
    navmesh_grid_get_cell_index,
    navmesh_grid_get_cell_neighbors,
    navmesh_grid_get_cell_filename,
    NAVMESH_STANDALONE_CELL_INDEX,
    navmesh_poly_update_flags,
    quantize,
)
from .navmesh_attributes import NavMeshAttr
from .. import logger

def export_ynv(navmesh_obj: Object, filepath: str) -> bool:    
    assert navmesh_is_valid(navmesh_obj)

    navmesh_xml = navmesh_from_object(navmesh_obj)
    if navmesh_xml is not None:
        navmesh_xml.write_xml(filepath)
        return True
    else:
        return False


def navmesh_from_object(navmesh_obj: Object) -> Optional[Navmesh]:
    """Create a ``NavMesh`` cwxml object."""

    is_standalone = navmesh_is_standalone(navmesh_obj)
    bbmin = get_min_vector_list(navmesh_obj.bound_box)
    bbmax = get_max_vector_list(navmesh_obj.bound_box)

    navmesh_xml = Navmesh()

    if is_standalone:
        navmesh_xml.bb_min = bbmin
        navmesh_xml.bb_max = bbmax
        navmesh_xml.bb_size = bbmax - bbmin
        navmesh_xml.area_id = NAVMESH_STANDALONE_CELL_INDEX
    else:
        cell_x, cell_y = navmesh_get_grid_cell(navmesh_obj)
        cell_min, cell_max = navmesh_grid_get_cell_bounds(cell_x, cell_y)
        cell_min.z = bbmin.z
        cell_max.z = bbmax.z
        navmesh_xml.bb_min = cell_min
        navmesh_xml.bb_max = cell_max
        navmesh_xml.bb_size = cell_max - cell_min
        navmesh_xml.area_id = navmesh_grid_get_cell_index(cell_x, cell_y)

    navmesh_xml.polygons, has_water, is_dlc = polygons_from_object(navmesh_obj)
    navmesh_xml.adj_area_ids = " ".join(map(str, get_adj_area_ids(navmesh_obj)))
    navmesh_xml.build_id = navmesh_obj.get("build_id", 0)

    links_obj = next((c for c in navmesh_obj.children if c.sollum_type == SollumType.NAVMESH_LINK_GROUP), None)
    cover_points_obj = next((c for c in navmesh_obj.children if c.sollum_type == SollumType.NAVMESH_COVER_POINT_GROUP), None)

    if links_obj:
        links_objs = [c for c in links_obj.children if c.sollum_type == SollumType.NAVMESH_LINK]
        links_objs.sort(key=lambda o: int(''.join(filter(str.isdigit, o.name)) or 0))
        for link_obj in links_objs:
                link_xml = link_from_object(link_obj)
                if link_xml:
                    navmesh_xml.links.append(link_xml)

    if cover_points_obj:
        for cover_point_obj in cover_points_obj.children:
            if cover_point_obj.sollum_type == SollumType.NAVMESH_COVER_POINT:
                navmesh_xml.cover_points.append(cover_point_from_object(cover_point_obj))

    content_flags = ["Polygons"]
    if navmesh_xml.links:
        content_flags.append("SpecialLinks")
    if is_standalone:
        content_flags.append("Vehicle")
    if has_water:
        content_flags.append("Unknown8")
    navmesh_xml.content_flags = ", ".join(content_flags)

    return navmesh_xml


def locate_neighbors_in_scene(navmesh_obj: Object) -> Optional[dict[tuple[int, int], Object]]:
    cell_to_obj = {navmesh_get_grid_cell(obj): obj for obj in bpy.context.scene.objects if navmesh_is_map(obj)}

    cell_x, cell_y = navmesh_get_grid_cell(navmesh_obj)
    cells = {}
    missing_cells = []
    for ncell_x, ncell_y in navmesh_grid_get_cell_neighbors(cell_x, cell_y):
        ncell = cell_to_obj.get((ncell_x, ncell_y), None)
        if ncell is not None:
            cells[(ncell_x, ncell_y)] = ncell
        else:
            missing_cells.append((ncell_x, ncell_y))

    if len(missing_cells) > 0:
        missing_cells_str = ", ".join(navmesh_grid_get_cell_filename(x, y) for x, y in missing_cells)
        logger.error(
            "Map navmesh export requires neighboring cells. "
            f"The following navmeshes must be imported into the scene: {missing_cells_str}."
        )
        return None
    else:
        return cells


def link_from_object(link_obj: Object) -> Optional[NavLink]:
    """Convert a single NavLink object to a NavLink XML/data instance."""
    assert link_obj.sollum_type == SollumType.NAVMESH_LINK

    link_target_obj = next((c for c in link_obj.children if c.sollum_type == SollumType.NAVMESH_LINK_TARGET), None)
    if link_target_obj is None:
        logger.error(f"Link '{link_obj.name}' has no target object!")
        return None

    link_props = link_obj.sz_nav_link
    link_xml = NavLink()
    link_xml.type = link_props.link_type
    link_xml.angle = wrap_angle(link_props.heading)
    link_xml.position_from = link_obj.location
    link_xml.position_to = link_obj.location + link_target_obj.location
    link_xml.area_from = link_props.area_from
    link_xml.area_to = link_props.area_to
    link_xml.poly_from = link_props.poly_from
    link_xml.poly_to = link_props.poly_to
    return link_xml


def cover_point_from_object(cover_point_obj: Object) -> NavPoint:
    assert cover_point_obj.sollum_type == SollumType.NAVMESH_COVER_POINT

    cover_point_props = cover_point_obj.sz_nav_cover_point
    cover_point_xml = NavPoint()
    cover_point_xml.type = cover_point_props.get_raw_int()
    cover_point_xml.angle = wrap_angle(cover_point_obj.rotation_euler.z - math.pi)
    cover_point_xml.position = Vector(cover_point_obj.location)
    return cover_point_xml


def build_edge_neighbors(mesh: bpy.types.Mesh, step=0.05) -> dict[tuple[int, int], int]:
    """Precompute polygon edge neighbors for the whole mesh"""
    verts = [v.co.copy() for v in mesh.vertices]
    edge_map = defaultdict(list)

    for poly in mesh.polygons:
        n = len(poly.vertices)
        if n < 3:
            continue
        for i in range(n):
            vi0 = poly.vertices[i]
            vi1 = poly.vertices[(i + 1) % n]
            v0 = verts[vi0]
            v1 = verts[vi1]
            key = tuple(sorted((quantize(v0, step), quantize(v1, step))))
            edge_map[key].append((poly.index, i))

    edge_neighbors = {}
    for polys in edge_map.values():
        if len(polys) == 2:
            (p0, i0), (p1, i1) = polys
            edge_neighbors[(p0, i0)] = p1
            edge_neighbors[(p1, i1)] = p0
        else:
            for p_idx, i in polys:
                edge_neighbors[(p_idx, i)] = 32767

    return edge_neighbors


def find_connected_components(mesh: bpy.types.Mesh, step=0.05):
    """Return list of connected polygon sets (each set is indices of connected polys)."""
    edge_neighbors = build_edge_neighbors(mesh, step)
    visited = set()
    components = []

    for poly in mesh.polygons:
        if poly.index in visited:
            continue

        comp = set()
        stack = [poly.index]

        while stack:
            p_idx = stack.pop()
            if p_idx in visited:
                continue
            visited.add(p_idx)
            comp.add(p_idx)

            n_verts = len(mesh.polygons[p_idx].vertices)
            for i in range(n_verts):
                neighbor = edge_neighbors.get((p_idx, i))
                if neighbor is not None and neighbor != 32767:
                    if neighbor not in visited:
                        stack.append(neighbor)

        components.append(comp)
    return components


def compute_space_around_vertex(v: Vector, bvh, max_radius=8.0, directions=8):
    """Approximate free space radius around a vertex using raycasts"""
    min_dist = max_radius
    for i in range(directions):
        angle = (2 * math.pi * i) / directions
        dir2d = Vector((math.cos(angle), math.sin(angle), 0.0))
        location, normal, index, dist = bvh.ray_cast(v, dir2d, max_radius)
        if location is not None:
            min_dist = min(min_dist, dist)
    return min_dist


def mark_isolated_polys(mesh, polygons_xml, blender_to_xml, min_area=64.0):
    """Mark polys that are in isolated patches of navmesh."""
    comps = find_connected_components(mesh)  # Flood fill / DFS
    for comp in comps:
        total_area = sum(mesh.polygons[idx].area for idx in comp)
        is_isolated = total_area < min_area

        if is_isolated:
            for idx in comp:
                if idx not in blender_to_xml:
                    continue
                poly_xml = polygons_xml[blender_to_xml[idx]]

                if "Isolated" not in poly_xml.flags:
                    if poly_xml.flags and poly_xml.flags != "None":
                        poly_xml.flags = poly_xml.flags + ", Isolated"
                    else:
                        poly_xml.flags = "Isolated"
    return polygons_xml


def mark_high_drops(mesh, polygons_xml, blender_to_xml, bvh):
    """Attempt to mark high-drop edges and add DangerKeepAway1 to polys."""

    edge_to_polys = {}
    for poly in mesh.polygons:
        for edge in poly.edge_keys:
            edge_to_polys.setdefault(edge, []).append(poly.index)
    danger_polys = set()

    # Detect cliffs
    for poly in mesh.polygons:
        if poly.index not in blender_to_xml:
            continue

        poly_xml = polygons_xml[blender_to_xml[poly.index]]   
        edges = poly_xml.edges.splitlines()
        edges_changed = False
        cliff_found = False

        verts = [mesh.vertices[v].co for v in poly.vertices]
        for loop_idx in range(len(verts)):
            v1 = verts[loop_idx]
            v2 = verts[(loop_idx + 1) % len(verts)]
            mid = (v1 + v2) * 0.5

            # Raycast downward from just below the edge
            hit, _, _, _ = bvh.ray_cast(mid + Vector((0, 0, 0.1)), Vector((0, 0, -1)), 50.0)  # 50m drop check
            dz = mid.z - hit.z if hit is not None else 999.0

            if dz > 3.5:
                cliff_found = True
                if len(edges) > loop_idx:
                    parts = edges[loop_idx].split(";")
                    if len(parts) == 4:
                        parts[3] = "HighDropOverEdge"
                        edges[loop_idx] = ";".join(parts)
                        edges_changed = True
        
        if cliff_found:
            danger_polys.add(poly.index)
            if edges_changed:
                poly_xml.edges = "\n".join(edges)
    
    # Propagate to neighbors
    frontier = set(danger_polys)
    visited = set(danger_polys)

    for _ in range(2): # Propagation depth
        new_frontier = set()
        for poly_idx in frontier:
            poly = mesh.polygons[poly_idx]
            for edge in poly.edge_keys:
                for neigh_idx in edge_to_polys.get(edge, []):
                    if neigh_idx not in visited:
                        visited.add(neigh_idx)
                        new_frontier.add(neigh_idx)
        frontier = new_frontier
        danger_polys.update(new_frontier)

    # Apply flags to all marked polys
    for poly_idx in danger_polys:
        if poly_idx in blender_to_xml:
            poly_xml = polygons_xml[blender_to_xml[poly_idx]]
            if "DangerKeepAway1" not in poly_xml.flags:
                poly_xml.flags = (
                    poly_xml.flags + ", DangerKeepAway1"
                    if poly_xml.flags and poly_xml.flags != "None"
                    else "DangerKeepAway1"
                )

    return polygons_xml


def compute_edge_flags_struct(poly, mesh, bvh, edge_area_id, edge_poly_id, edge_space_around_vertex, edge_flags, nav_area_id, edge_neighbors, first_adj_area_id_ref, poly_index_to_xml_id, recompute=False):
    """Build edge flags for one polygon, using precomputed edge neighbors"""

    edge_flags_struct = []
    for loop_idx, v in enumerate(poly.loop_indices):
        area_id = int(edge_area_id[v].value)
        poly_id = int(edge_poly_id[v].value)
        space_around_vertex = int(edge_space_around_vertex[v].value) if edge_space_around_vertex else 0
        flags = int(edge_flags[v].value)

        # Neighbor fixup
        if recompute or poly_id <= 0 or poly_id > 32767:
            neighbor_poly = edge_neighbors.get((poly.index, loop_idx), 32767)
            if neighbor_poly != 32767:
                poly_id = poly_index_to_xml_id.get(neighbor_poly, 32767)
            else:
                poly_id = 32767

        # Area ID fixup
        if area_id <= 0 or area_id == 0xFFFFFFFF:
            if poly_id == 32767:
                area_id = 65535
            else:
                area_id = nav_area_id
                first_adj_area_id_ref[0] = nav_area_id

        # Space fixup
        if recompute:
            v_index = poly.vertices[loop_idx]
            v_co = mesh.vertices[v_index].co
            radius = compute_space_around_vertex(v_co, bvh)
            space_around_vertex = min(max(int(round(radius * 4.0)), 0), 31)
        elif space_around_vertex < 0 or space_around_vertex > 31:
                space_around_vertex = 0

        edge_flags_struct.append((area_id, poly_id, space_around_vertex, flags))

    return edge_flags_struct


def polygons_from_object(navmesh_obj: Object) -> tuple[list[NavPolygon], bool]:
    """Convert Blender navmesh object to NavPolygon list."""
    assert navmesh_is_valid(navmesh_obj)

    polygons_xml = []
    has_water = False
    is_dlc = False

    # Get cell/area IDs
    cell_x, cell_y = navmesh_get_grid_cell(navmesh_obj)
    nav_area_id = navmesh_grid_get_cell_index(cell_x, cell_y)

    # Get evaluated mesh
    navmesh_obj_eval = get_evaluated_obj(navmesh_obj)
    mesh = navmesh_obj_eval.to_mesh()
    poly_index_to_xml_id = {poly.index: xml_id for xml_id, poly in enumerate(mesh.polygons)}

    # Attributes
    poly_data0 = mesh.attributes[NavMeshAttr.POLY_DATA_0].data
    poly_data1 = mesh.attributes[NavMeshAttr.POLY_DATA_1].data
    poly_data2 = mesh.attributes[NavMeshAttr.POLY_DATA_2].data
    poly_data3 = mesh.attributes[NavMeshAttr.POLY_DATA_3].data

    edge_area_id = mesh.attributes[NavMeshAttr.EDGE_AREA_ID].data
    edge_poly_id = mesh.attributes[NavMeshAttr.EDGE_POLY_ID].data
    edge_space_around_vertex = mesh.attributes[NavMeshAttr.EDGE_SPACE_AROUND_VERTEX].data
    edge_flags = mesh.attributes[NavMeshAttr.EDGE_FLAGS].data
    audio_data = mesh.attributes[NavMeshAttr.AUDIO_DATA].data

    compute_edges = bpy.context.window_manager.sz_ui_nav_compute_edge_neighbors
    first_adj_area_id_ref = [0]
    poly_bbox_resolution = 0.25

    # Precompute neighbors (per poly edge loop)
    edge_neighbors = build_edge_neighbors(mesh)

    # For edge "space around" recompute
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bvh = BVHTree.FromBMesh(bm)
    bm.free()

    def is_valid_poly(verts, eps=1e-5):
        unique = { (round(v.x,4), round(v.y,4), round(v.z,4)) for v in verts }
        if len(unique) < 3:
            return False
        u = list(unique)
        normal = (Vector(u[1]) - Vector(u[0])).cross(Vector(u[2]) - Vector(u[0]))
        return normal.length > eps

    for idx, poly in enumerate(mesh.polygons):
        poly_verts = [navmesh_obj.matrix_world @ mesh.vertices[v].co for v in poly.vertices]

        # Skip degenerate polys
        if not is_valid_poly(poly_verts):
            continue
        
        # Centroid + compressed bbox coords
        centroid = sum(poly_verts, Vector()) / len(poly_verts)
        poly_min = get_min_vector_list(poly_verts)
        poly_max = get_max_vector_list(poly_verts)

        poly_min_q = Vector((math.floor(poly_min.x / poly_bbox_resolution) * poly_bbox_resolution,
                            math.floor(poly_min.y / poly_bbox_resolution) * poly_bbox_resolution,
                            math.floor(poly_min.z / poly_bbox_resolution) * poly_bbox_resolution))
        poly_max_q = Vector((math.ceil(poly_max.x / poly_bbox_resolution) * poly_bbox_resolution,
                            math.ceil(poly_max.y / poly_bbox_resolution) * poly_bbox_resolution,
                            math.ceil(poly_max.z / poly_bbox_resolution) * poly_bbox_resolution))

        poly_size_q = poly_max_q - poly_min_q
        rel_x = (centroid.x - poly_min_q.x) / poly_size_q.x if poly_size_q.x != 0 else 0.0
        rel_y = (centroid.y - poly_min_q.y) / poly_size_q.y if poly_size_q.y != 0 else 0.0
        compressed_centroid_x = min(max(int(rel_x * 256.0), 0), 255)
        compressed_centroid_y = min(max(int(rel_y * 256.0), 0), 255)

        # Edge flags
        edge_flags_struct = compute_edge_flags_struct(
            poly,
            mesh,
            bvh,
            edge_area_id,
            edge_poly_id,
            edge_space_around_vertex,
            edge_flags,
            nav_area_id,
            edge_neighbors if edge_neighbors else {},
            first_adj_area_id_ref,
            poly_index_to_xml_id,
            compute_edges
        )

        # Poly flags & audio
        flag0 = int(poly_data0[poly.index].value)
        flag1 = int(poly_data1[poly.index].value)
        flag2 = int(poly_data2[poly.index].value)
        flag3 = int(poly_data3[poly.index].value)
        audio = int(audio_data[poly.index].value)

        if int(flag1) & 0x80:
            is_dlc = True

        # Build NavPolygon
        poly_xml = NavPolygon()
        poly_xml.vertices = poly_verts
        poly_xml.audio_data = audio
        poly_xml.centroid_x = compressed_centroid_x
        poly_xml.centroid_y = compressed_centroid_y
        poly_xml.edges = "\n".join(f"{e[0]}; {e[1]}; {e[2]}; {e[3]}" for e in edge_flags_struct)
        poly_xml.flags = Navmesh.flags_to_names(flag0, flag1, flag2, flag3)  
        polygons_xml.append(poly_xml)
        
    navmesh_obj_eval.to_mesh_clear()

    if compute_edges and first_adj_area_id_ref[0] > 0:
        if "adj_area_ids" not in navmesh_obj or not navmesh_obj["adj_area_ids"]:
            navmesh_obj["adj_area_ids"] = [first_adj_area_id_ref[0]]
        else:
            navmesh_obj["adj_area_ids"][0] = first_adj_area_id_ref[0]

    return polygons_xml, has_water, is_dlc