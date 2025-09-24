import sys
import time
import bpy
from bpy.types import (Context, Operator)
from bpy.props import (BoolProperty)
import bmesh
from .ynvimport import navmesh_to_obj
from ..cwxml.navmesh import Navmesh
from ..sollumz_properties import SollumType
from .navmesh import (collision_to_navmesh, gather_bounds, navmesh_is_valid, navmesh_poly_update_flags,)
from .navmesh_attributes import (NavPolyAttributes, mesh_iter_navmesh_all_poly_attributes,)
from ..tools.blenderhelper import tag_redraw
from mathutils import Vector, Matrix


class SOLLUMZ_OT_navmesh_polys_update_flags(Operator):
    bl_idname = "sollumz.navmesh_polys_update_flags"
    bl_label = "Update Poly Flags"
    bl_description = "Update 'Small' and 'Large' flags"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(self, context):
        self.poll_message_set("Must be in Edit Mode.")
        return context.mode == "EDIT_MESH"

    def execute(self, context: Context):
        for obj in context.objects_in_mode:
            if not navmesh_is_valid(obj):
                continue

            mesh = obj.data
            poly_access = mesh.sz_navmesh_poly_access
            navmesh_poly_update_flags(mesh, poly_access.active_poly)
            for poly_idx in poly_access.selected_polys:
                navmesh_poly_update_flags(mesh, poly_idx)

            mesh.update_tag()

        tag_redraw(bpy.context, space_type="VIEW_3D", region_type="WINDOW")
        return {"FINISHED"}


class SOLLUMZ_OT_navmesh_polys_select_similar(Operator):
    bl_idname = "sollumz.navmesh_polys_select_similar"
    bl_label = "Select Similar Polygons"
    bl_description = "Select polygons with same attributes as the active polygon"
    bl_options = {"REGISTER", "UNDO"}

    use_is_small: BoolProperty(name="Small", default=False)
    use_is_large: BoolProperty(name="Large", default=False)
    use_is_unk3: BoolProperty(name="Unk3", default=False)
    use_is_sheltered: BoolProperty(name="Sheltered", default=False)
    use_is_unused10: BoolProperty(name="Unused10", default=False)
    use_is_unused20: BoolProperty(name="Unused20", default=False)
    use_is_switched_off_for_peds: BoolProperty(name="Switched Off For Peds", default=False)
    use_is_water: BoolProperty(name="Water", default=False)
    use_is_danger_keep_away1: BoolProperty(name="Danger Keep Away 1", default=False)
    use_is_danger_keep_away2: BoolProperty(name="Danger Keep Away 2", default=False)
    use_is_sheltered2: BoolProperty(name="Sheltered 2", default=False)
    use_is_unk12: BoolProperty(name="Unk12", default=False)
    use_is_near_car_node: BoolProperty(name="Near Car Node", default=False)
    use_is_interior: BoolProperty(name="Interior", default=False)
    use_is_isolated: BoolProperty(name="Isolated", default=False)
    use_is_zero_area_stitch: BoolProperty(name="Zero Area Stitch", default=False)
    use_is_network_spawn_candidate: BoolProperty(name="Network Spawn Candidate", default=False)
    use_is_road: BoolProperty(name="Road", default=False)
    use_is_lies_along_edge: BoolProperty(name="Lies Along Edge", default=False)
    use_is_unk21: BoolProperty(name="Unk21", default=False)
    use_is_unk22: BoolProperty(name="Unk22", default=False)
    use_is_unk23: BoolProperty(name="Unk23", default=False)
    use_is_muddy: BoolProperty(name="Muddy", default=False)
    use_is_sheltered3: BoolProperty(name="Sheltered 3", default=False)
    use_is_vegetation: BoolProperty(name="Vegetation", default=False)
    use_is_boardwalk: BoolProperty(name="Boardwalk", default=False)
    use_is_pavement: BoolProperty(name="Pavement", default=False)
    use_is_unk29: BoolProperty(name="Unk29", default=False)
    use_is_door: BoolProperty(name="Door", default=False)
    use_is_stairs: BoolProperty(name="Stairs", default=False)
    use_is_steep_slope: BoolProperty(name="Steep Slope", default=False)

    @classmethod
    def poll(self, context):
        self.poll_message_set("Must be in Edit Mode.")
        return context.mode == "EDIT_MESH" and navmesh_is_valid(context.active_object)

    def execute(self, context: Context):
        aobj = context.active_object
        target_poly_attrs = aobj.data.sz_navmesh_poly_access.active_poly_attributes
        fields_to_consider = self._get_fields_to_consider()

        for obj in context.objects_in_mode:
            if not navmesh_is_valid(obj):
                continue

            mesh = obj.data
            if mesh.is_editmode:
                bm = bmesh.from_edit_mesh(mesh)
                try:
                    bm.faces.ensure_lookup_table()
                    for poly_idx, poly_attrs in enumerate(mesh_iter_navmesh_all_poly_attributes(mesh)):
                        if self._is_similar(poly_attrs, target_poly_attrs, fields_to_consider):
                            bm.faces[poly_idx].select = True
                finally:
                    bm.free()
            else:
                polys = mesh.polygons
                for poly_idx, poly_attrs in enumerate(mesh_iter_navmesh_all_poly_attributes(mesh)):
                    if self._is_similar(poly_attrs, target_poly_attrs, fields_to_consider):
                        polys[poly_idx].select = True

            mesh.update_tag()

        tag_redraw(bpy.context, space_type="VIEW_3D", region_type="WINDOW")
        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        col = layout.column(heading="Consider", align=True)
        for f in self.__annotations__.keys():
            col.prop(self, f)

    def _is_similar(self, a: NavPolyAttributes, b: NavPolyAttributes, fields: list[str]):
        for field in fields:
            if getattr(a, field) != getattr(b, field):
                return False

        return True

    def _get_fields_to_consider(self) -> list[str]:
        return [f[4:] for f in self.__annotations__.keys() if f.startswith("use_") and getattr(self, f)]
    

class NAVMESH_OT_generate_from_collision(Operator):
    bl_idname = "navmesh.generate_from_collision"
    bl_label = "Generate Navmesh from Collision"
    bl_description = "Generates navmeshes from selected YBN's (can take time...)"

    def execute(self, context):
        start_time = time.time()
        selected_objs = context.selected_objects

        if not selected_objs:
            self.report({'WARNING'}, "No objects selected")
            return {"CANCELLED"}

        generated_total = 0
        GRID_MIN_X, GRID_MIN_Y = -15900.0, -15900.0
        CELL_SIZE = 150.0

        all_bounds = []
        for obj in selected_objs:
            gather_bounds(obj, all_bounds)

        if not all_bounds:
            self.report({'WARNING'}, "No geometry found in selected object(s).")
            return {"CANCELLED"}

        sel_min_x = min(v.x for v in all_bounds)
        sel_max_x = max(v.x for v in all_bounds)
        sel_min_y = min(v.y for v in all_bounds)
        sel_max_y = max(v.y for v in all_bounds)

        min_cell_x = int((sel_min_x - GRID_MIN_X) // CELL_SIZE)
        max_cell_x = int((sel_max_x - GRID_MIN_X) // CELL_SIZE)
        min_cell_y = int((sel_min_y - GRID_MIN_Y) // CELL_SIZE)
        max_cell_y = int((sel_max_y - GRID_MIN_Y) // CELL_SIZE)

        for cell_x in range(min_cell_x, max_cell_x + 1):
            for cell_y in range(min_cell_y, max_cell_y + 1):
                cell_min_x = GRID_MIN_X + cell_x * CELL_SIZE
                cell_max_x = cell_min_x + CELL_SIZE
                cell_min_y = GRID_MIN_Y + cell_y * CELL_SIZE
                cell_max_y = cell_min_y + CELL_SIZE

                navmesh_xml = collision_to_navmesh(selected_objs, cell_min_x, cell_max_x, cell_min_y, cell_max_y)
                if navmesh_xml and navmesh_xml.polygons:
                    navmesh_name = f"navmesh_{cell_x}_{cell_y}"
                    navmesh_to_obj(navmesh_xml, navmesh_name)
                    generated_total += 1

        elapsed = time.time() - start_time
        self.report({'INFO'}, f"Generation complete. {generated_total} navmesh(es) created in {elapsed:.2f} seconds.")
        return {"FINISHED"}