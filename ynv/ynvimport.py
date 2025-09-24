import math
from typing import Sequence
from bpy.types import (
    Object,
    Mesh
)
import bpy
import bmesh
import numpy as np
from .navmesh import navmesh_compute_cell, navmesh_grid_get_cell_filename, navmesh_is_map
from .navmesh_material import get_navmesh_material
from .navmesh_attributes import NavMeshAttr, mesh_add_navmesh_attribute
from ..tools.meshhelper import create_box
from ..cwxml.navmesh import YNV, NavLink, NavPolygon, Navmesh
from ..sollumz_properties import SOLLUMZ_UI_NAMES, SollumType
import os
from ..tools.blenderhelper import find_bsdf_and_material_output


def points_to_obj(points):
    pobj = bpy.data.objects.new("Points", None)
    pobj.empty_display_size = 0

    for idx, point in enumerate(points):
        mesh = bpy.data.meshes.new(SOLLUMZ_UI_NAMES[SollumType.NAVMESH_POINT])
        obj = bpy.data.objects.new(
            SOLLUMZ_UI_NAMES[SollumType.NAVMESH_POINT] + " " + str(idx), mesh)
        obj.sollum_type = SollumType.NAVMESH_POINT
        obj.parent = pobj

        # properties
        create_box(mesh, 0.5)
        obj.location = point.position
        obj.rotation_euler = (0, 0, point.angle)
        bpy.context.collection.objects.link(obj)

    return pobj

 
def links_to_obj(links: Sequence[NavLink]) -> Object:
    pobj = bpy.data.objects.new("SpecialLinks", None)
    pobj.sollum_type = SollumType.NAVMESH_LINK_GROUP
    pobj.empty_display_size = 0

    for idx, link in enumerate(links):
        from_obj = bpy.data.objects.new(f"Link {idx}", None)
        from_obj.sollum_type = SollumType.NAVMESH_LINK
        from_obj.empty_display_size = 0.65
        from_obj.empty_display_type = "SPHERE"
        from_obj.parent = pobj
        from_obj.location = link.position_from
        bpy.context.collection.objects.link(from_obj)

        from_obj.sz_nav_link.link_type = link.type
        from_obj.sz_nav_link.heading = (link.angle / 255) * 2 * math.pi
        from_obj.sz_nav_link.area_from = link.area_from
        from_obj.sz_nav_link.area_to = link.area_to
        from_obj.sz_nav_link.poly_from = link.poly_from
        from_obj.sz_nav_link.poly_to = link.poly_to

        to_obj = bpy.data.objects.new(f"Link {idx}.target", None)
        to_obj.sollum_type = SollumType.NAVMESH_LINK_TARGET
        to_obj.empty_display_size = 0.45
        to_obj.empty_display_type = "SPHERE"
        to_obj.parent = from_obj
        to_obj.location = link.position_to - link.position_from
        bpy.context.collection.objects.link(to_obj)

    return pobj


def parse_navpoly_flags(flag_text: str) -> int:
    """Convert a <Flags> string like to the combined bitmask."""
    value = 0

    if not flag_text:
        return value
    
    # split by commas and whitespace
    parts = [p.strip() for p in flag_text.replace(",", " ").split()]
    for part in parts:
        if part in Navmesh.polygon_flags:
            value |= Navmesh.polygon_flags[part]
        else:
            try: # fallback: maybe old integer flags
                value |= int(part)
            except ValueError:
                print(f"Unknown flag: {part}")
    return value


def parse_navpoly_flags_tuple(flag_text: str) -> tuple[int, int, int, int]:
    """Convert <Flags> text into 4 packed parts (Flags1–Flags4)"""
    mask = parse_navpoly_flags(flag_text)

    f1 = mask & 0xFF
    f2 = (mask >> 8) & 0xFFF
    f3 = (mask >> 20) & 0xFFF
    f4 = (mask >> 32) & 0xF

    return f1, f2, f3, f4


def get_material(flags_str, material_cache, water_depth):
    if flags_str is None:
        flags_str = ""

    flags_str = str(flags_str)
    flag_names = [p.strip() for p in flags_str.split(",") if p.strip()]
    mat_name = "NavPoly " + ", ".join(flag_names)

    if mat_name in material_cache:
        return material_cache[mat_name]

    # Start from navmesh debug material instead of a blank new one
    base_mat = get_navmesh_material()
    mat = base_mat.copy()
    mat.name = mat_name

    r, g, b = 0.05, 0.05, 0.05

    # Map flag names to colors
    for fname in flag_names:
        if fname == "DangerKeepAway1":
            r += 0.25
        elif fname == "DangerKeepAway2":
            r += 0.25
        elif fname == "Pavement":
            g += 0.25
        elif fname == "Door":
            r += 0.5
            b += 0.5
        elif fname == "Stairs":
            r += 0.5
            g += 0.25
        elif fname == "Water":
            b += 0.15 * (8 - water_depth)
        elif fname == "Muddy":
            r += 0.2
            g += 0.1
        elif fname == "Interior":
            b += 0.1
        elif fname == "Road":
            b += 0.1
        elif fname == "Isolated":
            r += 0.25

    bsdf, _ = find_bsdf_and_material_output(mat)
    bsdf.inputs[0].default_value = (r, g, b, 0.75)

    material_cache[mat_name] = mat
    return mat


def polygons_to_mesh(name: str, polygons: Sequence[NavPolygon]) -> Mesh:
    material_cache = {}
    mats = []
    vertices = []
    faces = []

    poly_data_attrs = (NavMeshAttr.POLY_DATA_0, NavMeshAttr.POLY_DATA_1, NavMeshAttr.POLY_DATA_2, NavMeshAttr.POLY_DATA_3)
    audio_data_attrs = [NavMeshAttr.AUDIO_DATA]
    edge_data_attrs = (NavMeshAttr.EDGE_AREA_ID, NavMeshAttr.EDGE_POLY_ID, NavMeshAttr.EDGE_SPACE_AROUND_VERTEX, NavMeshAttr.EDGE_FLAGS)

    pending_poly_flags = [] # Per poly (f1..f4)
    pending_audio = [] # Per poly
    pending_edges = [] # Per loop: (area, polyid, space, flags)
    
    # Build verts/faces + collect attributes in the same order
    for poly in polygons:
        mats.append(get_material(poly.flags, material_cache, poly.water_depth))

        # Face indices into vertices
        face_indices = []
        for vert in poly.vertices:
            vert.freeze()
            face_indices.append(len(vertices))
            vertices.append(vert)

        flags_mask = parse_navpoly_flags(poly.flags)
        if len(face_indices) == 2:
            assert (flags_mask & Navmesh.polygon_flags["ZeroAreaStitchPolyDLC"]) != 0, \
                "Polygon with 2 vertices that is not a DLC stitch poly"     
            face_indices.append(face_indices[-1])

        faces.append(face_indices)
        f1,f2,f3,f4 = parse_navpoly_flags_tuple(poly.flags)
        pending_poly_flags.append((f1,f2,f3,f4))
        pending_audio.append(poly.audio_data)

        if isinstance(poly.edges, str):
            items = []
            for line in poly.edges.splitlines():
                a, b, c, d = [p.strip() for p in line.split(";")]
                try: # Last item may be a name like "HighDropOverEdge", map to int
                    d = int(d)
                except ValueError:
                    d = Navmesh.edge_flags.get(d, 0)
                items.append((int(a), int(b), int(c), int(d)))
            edge_items = items
        else:
            edge_items = list(poly.edges)

        # Push one edge item per face vertex, in order
        if len(edge_items) != len(face_indices):
            edge_items = (edge_items + [(65535, 32767, 0, 0)] * len(face_indices))[:len(face_indices)]
        pending_edges.extend(edge_items)

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)

    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.001)
    bm.to_mesh(mesh)
    bm.free()

    poly_count = len(mesh.polygons)
    loop_count = len(mesh.loops)

    poly_flags_array = np.zeros((poly_count, len(poly_data_attrs)), dtype=np.float64)
    audio_array = np.zeros(poly_count, dtype=np.uint32)
    edge_arrays = np.zeros((loop_count, len(edge_data_attrs)), dtype=np.uint32)

    for poly_idx, poly in enumerate(mesh.polygons):
        if poly_idx < len(polygons):
            nav_poly = polygons[poly_idx]

            f1,f2,f3,f4 = parse_navpoly_flags_tuple(nav_poly.flags)
            poly_flags_array[poly_idx, :] = (f1,f2,f3,f4)
            audio_array[poly_idx] = nav_poly.audio_data

            src_edges = nav_poly.edges or []
            for li, loop_index in enumerate(poly.loop_indices):
                if li < len(src_edges):
                    edge_arrays[loop_index, :] = src_edges[li]

    # Poly attributes
    for i, attr_name in enumerate(poly_data_attrs):
        attr = mesh.attributes.get(attr_name) or mesh.attributes.new(attr_name, 'FLOAT', 'FACE')
        attr.data.foreach_set("value", poly_flags_array[:, i].ravel())

    attr = mesh.attributes.get(NavMeshAttr.AUDIO_DATA) or mesh.attributes.new(NavMeshAttr.AUDIO_DATA, 'INT', 'FACE')
    attr.data.foreach_set("value", audio_array.ravel())

    # Edge attributes
    for i, attr_name in enumerate(edge_data_attrs):
        attr = mesh.attributes.get(attr_name) or mesh.attributes.new(attr_name, 'INT', 'CORNER')
        attr.data.foreach_set("value", edge_arrays[:, i].ravel())

    used_materials = []
    for mat in mats:
        if mat not in used_materials:
            mesh.materials.append(mat)
            used_materials.append(mat)

    for idx, poly in enumerate(mesh.polygons):
        poly.material_index = used_materials.index(mats[idx])

    return mesh


def navmesh_to_obj(navmesh, filepath):
    name = os.path.basename(filepath.replace(YNV.file_extension, ""))
    
    # Create main mesh object
    mesh = polygons_to_mesh(name, navmesh.polygons)
    mesh_obj = bpy.data.objects.new(name, mesh)
    mesh_obj.sollum_type = SollumType.NAVMESH
    mesh_obj.empty_display_size = 0
    bpy.context.collection.objects.link(mesh_obj)

    # Store data
    mesh_obj["adj_area_ids"] = [int(x) for x in navmesh.adj_area_ids.split()]
    mesh_obj["build_id"] = navmesh.build_id

    # Create links/portals object
    npobj = links_to_obj(navmesh.links)
    npobj.parent = mesh_obj
    bpy.context.collection.objects.link(npobj)

    # Create points object
    npobj = points_to_obj(navmesh.points)
    npobj.parent = mesh_obj
    bpy.context.collection.objects.link(npobj)

    if not navmesh_is_map(mesh_obj):
        x, y = navmesh_compute_cell(mesh_obj)
        if x >= 0 and y >= 0:
            mesh_obj.name = navmesh_grid_get_cell_filename(x, y)

    return mesh_obj


def import_ynv(filepath):
    ynv_xml = YNV.from_xml_file(filepath)
    navmesh_to_obj(ynv_xml, filepath)
