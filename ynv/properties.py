import bpy
from bpy.types import (
    Object,
    Mesh,
    WindowManager,
    PropertyGroup,
    ShaderNode,
)
from bpy.props import (
    BoolProperty,
    BoolVectorProperty,
    IntProperty,
    FloatProperty,
    FloatVectorProperty,
    EnumProperty,
    PointerProperty,
)
import bmesh
from collections.abc import Iterator
from enum import IntEnum

from ..tools.blenderhelper import tag_redraw
from .navmesh_attributes import (
    mesh_get_navmesh_poly_attributes,
    mesh_set_navmesh_poly_attributes,
    NavPolyAttributes,
    mesh_get_navmesh_edge_attributes,
    mesh_set_navmesh_edge_attributes,
    NavEdgeAttributes,
)
from . import navmesh_material


class NavCoverType(IntEnum):
    LOW_WALL = 0
    LOW_WALL_TO_LEFT = 1
    LOW_WALL_TO_RIGHT = 2
    WALL_TO_LEFT = 3
    WALL_TO_RIGHT = 4
    WALL_TO_NEITHER = 5


NavCoverTypeEnumItems = tuple((enum.name, label, desc, enum.value) for enum, label, desc in (
    (NavCoverType.LOW_WALL, "Low Wall", "Behind low wall, can only shoot over the top"),
    (NavCoverType.LOW_WALL_TO_LEFT, "Low Wall To Left", "Behind low wall corner, can shoot over the top and to the right"),
    (NavCoverType.LOW_WALL_TO_RIGHT, "Low Wall To Right", "Behind low wall corner, can shoot over the top and to the left"),
    (NavCoverType.WALL_TO_LEFT, "Wall To Left", "Behind high wall corner, can only shoot to the right"),
    (NavCoverType.WALL_TO_RIGHT, "Wall To Right", "Behind high wall corner, can only shoot to the left"),
    (NavCoverType.WALL_TO_NEITHER, "Wall To Neither", "Behind thin high wall, can shoot to either the left or right sides"),
))


class NavCoverPointProps(PropertyGroup):
    cover_type: EnumProperty(name="Type", items=NavCoverTypeEnumItems, default=NavCoverType.LOW_WALL.name)
    disabled: BoolProperty(name="Disabled", default=False)

    def get_raw_int(self) -> int:
        cover_type_int = NavCoverType[self.cover_type].value
        disabled_int = 0x8 if self.disabled else 0
        return cover_type_int | disabled_int

    def set_raw_int(self, value):
        cover_type_int = value & 0x7
        if cover_type_int <= 5:
            self.cover_type = NavCoverType(cover_type_int).name
        else:
            # in case of corrupted out-of-range values, default to low-wall
            self.cover_type = NavCoverType.LOW_WALL.name
        self.disabled = (value & 0x8) != 0


class NavLinkType(IntEnum):
    CLIMB_LADDER = 1
    DESCEND_LADDER = 2
    CLIMB_OBJECT = 3


NavLinkTypeEnumItems = tuple((enum.name, label, desc, enum.value) for enum, label, desc in (
    (NavLinkType.CLIMB_LADDER, "Climb Ladder", "Link from the bottom of a ladder to the top"),
    (NavLinkType.DESCEND_LADDER, "Descend Ladder", "Link from the top of a ladder to the bottom"),
    (NavLinkType.CLIMB_OBJECT, "Climb Object", "Link for a climbable object")
))


class NavEdgeFlags(IntEnum):
    NONE = 0,
    ADJACENCY_DISABLED = 1
    EDGE_PROVIDES_COVER = 2
    HIGH_DROP_OVER_EDGE = 4
    EXTERNAL_EDGE = 8


NavmeshEdgeFlagsEnumItems = tuple((enum.name, label, desc, enum.value) for enum, label, desc in (
    (NavEdgeFlags.NONE, "None", ""),
    (NavEdgeFlags.ADJACENCY_DISABLED, "Adjacency Disabled", "Adjacency disabled (impossible to traverse during route-finding, climbs only)"),
    (NavEdgeFlags.EDGE_PROVIDES_COVER, "Provides Cover", "Edge length provides cover perpendicular to it"),
    (NavEdgeFlags.HIGH_DROP_OVER_EDGE, "High Drop", "High drop associated with non-standard adjacency"),
    (NavEdgeFlags.EXTERNAL_EDGE, "External Edge", "Polygon lies on navmesh border (candidate for adjacent navmesh connection)")
))


class NavLinkProps(PropertyGroup):
    link_type: IntProperty(name="Link Type") # Used to be 1, 2 or 3 for GTAV (NavLinkType) but in RDR2 the values goes up to 125
    heading: FloatProperty(name="Heading", subtype="ANGLE", unit="ROTATION")
    area_from: IntProperty(name="Area From")
    area_to: IntProperty(name="Area To")
    poly_from: IntProperty(name="Poly From")
    poly_to: IntProperty(name="Poly To")


def _edge_attr_getter(attr_name: str):
    def fn(self):
        return getattr(self.active_edge_attributes, attr_name)

    return fn


def _edge_attr_setter(attr_name: str):
    def fn(self, value):
        mesh = self.mesh
        for selected_edge in self.selected_edges:
            attrs = mesh_get_navmesh_edge_attributes(mesh, selected_edge)
            setattr(attrs, attr_name, value)
            mesh_set_navmesh_edge_attributes(mesh, selected_edge, attrs)

        active_edge = self.active_edge
        attrs = mesh_get_navmesh_edge_attributes(mesh, active_edge)
        setattr(attrs, attr_name, value)
        mesh_set_navmesh_edge_attributes(mesh, active_edge, attrs)

        mesh.update_tag()
        tag_redraw(bpy.context, space_type="VIEW_3D", region_type="WINDOW")

    return fn



def EdgeIntAttr(name: str, attr_name: str, min: int, max: int):
    return IntProperty(
        name=name,
        get=_edge_attr_getter(attr_name), set=_edge_attr_setter(attr_name),
        min=min, max=max,
    )


def EdgeEnumAttr(name: str, attr_name: str, items, default=None):
    return EnumProperty(
        name=name,
        items=items,
        default=default,
        get=_edge_attr_getter(attr_name),
        set=_edge_attr_setter(attr_name),
    )


class NavMeshEdgeAccessor(PropertyGroup):
    """Property group to allow to access navmesh edge attributes from the UI."""

    area_id: EdgeIntAttr("Navmesh Index", "area_id", min=0, max=0xFFFF)
    poly_id: EdgeIntAttr("Polygon Index", "poly_id", min=0, max=0xFF7F)
    space_around_vertex: EdgeIntAttr("Space Around Vertex", "space_around_vertex", min=0, max=0xFFFF)
    flags: EdgeEnumAttr("Flags", "flags", NavmeshEdgeFlagsEnumItems, default=NavEdgeFlags.NONE.name)

    @property
    def obj(self) -> bpy.types.Object:
        """Return the object this accessor is attached to."""
        return self.id_data
    
    @property
    def mesh(self) -> Mesh:
        return self.obj if self.obj else None

    @property
    def active_edge(self) -> int:
        mesh = self.mesh
        if mesh is None:
            return 0
        if mesh.is_editmode:
            bm = bmesh.from_edit_mesh(mesh)
            bm.edges.index_update()
            if bm.select_mode == {"CORNER"} and bm.select_history.active:
                return bm.select_history.active.index
        return 0

    @property
    def selected_edges(self) -> Iterator[int]:
        mesh = self.mesh
        if mesh is None:
            return []
        if mesh.is_editmode:
            bm = bmesh.from_edit_mesh(mesh)
            return [e.index for e in bm.edges if e.select]
        return []

    @property
    def active_edge_attributes(self) -> NavEdgeAttributes:
        return mesh_get_navmesh_edge_attributes(self.mesh, self.active_edge)


# Helper functions for NavMeshPolyAccessor
def _attr_getter(attr_name: str):
    def fn(self):
        return getattr(self.active_poly_attributes, attr_name)

    return fn


def _attr_setter(attr_name: str):
    def fn(self, value):
        mesh = self.mesh
        for selected_poly in self.selected_polys:
            attrs = mesh_get_navmesh_poly_attributes(mesh, selected_poly)
            setattr(attrs, attr_name, value)
            mesh_set_navmesh_poly_attributes(mesh, selected_poly, attrs)

        active_poly = self.active_poly
        attrs = mesh_get_navmesh_poly_attributes(mesh, active_poly)
        setattr(attrs, attr_name, value)
        mesh_set_navmesh_poly_attributes(mesh, active_poly, attrs)

        mesh.update_tag()
        tag_redraw(bpy.context, space_type="VIEW_3D", region_type="WINDOW")

    return fn


def BoolAttr(name: str, attr_name: str):
    return BoolProperty(
        name=name,
        get=_attr_getter(attr_name), set=_attr_setter(attr_name),
    )


def IntAttr(name: str, attr_name: str, min: int, max: int):
    return IntProperty(
        name=name,
        get=_attr_getter(attr_name), set=_attr_setter(attr_name),
        min=min, max=max,
    )


def BoolVectorAttr(name: str, attr_name: str, size: int):
    return BoolVectorProperty(
        name=name,
        size=size,
        get=_attr_getter(attr_name), set=_attr_setter(attr_name),
    )


class NavMeshPolyAccessor(PropertyGroup):
    """Property group to allow to access navmesh polygon attributes from the UI."""

    # ---------- Flags1 ----------
    is_small: BoolAttr("Small", "is_small")
    is_large: BoolAttr("Large", "is_large")
    is_unk3: BoolAttr("Unk3", "is_unk3")
    is_sheltered: BoolAttr("Sheltered", "is_sheltered")
    is_unused10: BoolAttr("Unused10", "is_unused10")
    is_unused20: BoolAttr("Unused20", "is_unused20")
    is_switched_off_for_peds: BoolAttr("Switched Off For Peds", "is_switched_off_for_peds")
    is_water: BoolAttr("Water", "is_water")

    # ---------- Flags2 ----------
    is_danger_keep_away1: BoolAttr("Danger Keep Away 1", "is_danger_keep_away1")
    is_danger_keep_away2: BoolAttr("Danger Keep Away 2", "is_danger_keep_away2")
    is_sheltered2: BoolAttr("Sheltered2", "is_sheltered2")
    is_unk12: BoolAttr("Unk12", "is_unk12")
    is_near_car_node: BoolAttr("Near Car Node", "is_near_car_node")
    is_interior: BoolAttr("Interior", "is_interior")
    is_isolated: BoolAttr("Isolated", "is_isolated")
    is_zero_area_stitch: BoolAttr("Zero Area Stitch Poly DLC", "is_zero_area_stitch")
    is_network_spawn_candidate: BoolAttr("Network Spawn Candidate", "is_network_spawn_candidate")
    is_road: BoolAttr("Road", "is_road")
    is_unk19: BoolAttr("Unk19", "is_unk19")
    lies_along_edge: BoolAttr("Lies Along Edge", "lies_along_edge")

    # ---------- Flags3 ----------
    is_unk21: BoolAttr("Unk21", "is_unk21")
    is_unk22: BoolAttr("Unk22", "is_unk22")
    is_unk23: BoolAttr("Unk23", "is_unk23")
    is_muddy: BoolAttr("Muddy", "is_muddy")
    is_sheltered3: BoolAttr("Sheltered3", "is_sheltered3")
    is_vegetation: BoolAttr("Vegetation", "is_vegetation")
    is_boardwalk: BoolAttr("Boardwalk", "is_boardwalk")
    is_pavement: BoolAttr("Pavement", "is_pavement")
    is_unk29: BoolAttr("Unk29", "is_unk29")
    is_door: BoolAttr("Door", "is_door")
    is_stairs: BoolAttr("Stairs", "is_stairs")
    is_steep_slope: BoolAttr("Steep Slope", "is_steep_slope")

    # ---------- Flags4 ----------
    is_unk33: BoolAttr("Unk33", "is_unk33")
    is_shelter4: BoolAttr("Shelter4", "is_shelter4")
    is_unk35: BoolAttr("Unk35", "is_unk35")
    is_unk36: BoolAttr("Unk36", "is_unk36")

    audio_property: IntAttr("Audio Property", "audio_property", min=0, max=15)
    is_dlc_stitch: BoolAttr("DLC Stitch", "is_dlc_stitch")

    @property
    def obj(self) -> bpy.types.Object:
        """Return the object this accessor is attached to."""
        return self.id_data
    
    @property
    def mesh(self) -> Mesh:
        return self.obj if self.obj else None

    @property
    def active_poly(self) -> int:
        mesh = self.mesh
        if mesh.is_editmode:
            bm = bmesh.from_edit_mesh(mesh)
            bm.faces.index_update()
            return bm.faces.active.index
        else:
            return mesh.polygons.active

    @property
    def selected_polys(self) -> Iterator[int]:
        mesh = self.mesh
        active_poly = self.active_poly
        if mesh.is_editmode:
            bm = bmesh.from_edit_mesh(mesh)
            bm.faces.index_update()
            for face in bm.faces:
                if face.index != active_poly and face.select:
                    yield face.index
        else:
            for poly in mesh.polygons:
                if poly.index != active_poly and poly.select:
                    yield poly.index

    @property
    def active_poly_attributes(self) -> NavPolyAttributes:
        return mesh_get_navmesh_poly_attributes(self.mesh, self.active_poly)


class NavMeshPolyRender(PropertyGroup):
    @property
    def obj(self) -> bpy.types.Object:
        """Return the object this accessor is attached to."""
        return self.id_data
    
    @property
    def mesh(self) -> Mesh:
        return self.obj if self.obj else None

    def get_node(self, name: str) -> ShaderNode:
        mesh = self.mesh
        if not mesh or not mesh.materials or not mesh.materials[0]:
            return None
        node_tree = mesh.materials[0].node_tree
        return node_tree.nodes.get(name, None)


def _define_poly_render_flag_properties(flag: navmesh_material.FlagRenderInfo):
    def _toggle_getter(self: NavMeshPolyRender) -> bool:
        node = self.get_node(flag.toggle_name)
        if node:
            return node and node.outputs[0].default_value != 0
        return False

    def _toggle_setter(self: NavMeshPolyRender, value: bool):
        node = self.get_node(flag.toggle_name)
        if node:
            node.outputs[0].default_value = 1.0 if value else 0.0

    def _color_getter(self: NavMeshPolyRender) -> tuple[float, float, float]:
        node = self.get_node(flag.color_name)
        if node:
            return tuple(node.inputs[i].default_value for i in range(3))
        return (0.0, 0.0, 0.0)

    def _color_setter(self: NavMeshPolyRender, value: tuple[float, float, float]):
        node = self.get_node(flag.color_name)
        if node:
            for i in range(3):
                node.inputs[i].default_value = value[i]

    NavMeshPolyRender.__annotations__[flag.toggle_name] = BoolProperty(
        name=flag.toggle_name,
        get=_toggle_getter,
        set=_toggle_setter,
    )
    NavMeshPolyRender.__annotations__[flag.color_name] = FloatVectorProperty(
        name=flag.color_name, size=3, subtype="COLOR", min=0.0, max=1.0,
        get=_color_getter,
        set=_color_setter,
    )


for flag in navmesh_material.ALL_FLAGS:
    _define_poly_render_flag_properties(flag)


def register():
    Object.sz_nav_cover_point = PointerProperty(type=NavCoverPointProps)
    Object.sz_nav_link = PointerProperty(type=NavLinkProps)
    Mesh.sz_navmesh_poly_access = PointerProperty(type=NavMeshPolyAccessor)
    Mesh.sz_navmesh_edge_access = PointerProperty(type=NavMeshEdgeAccessor)
    Mesh.sz_navmesh_poly_render = PointerProperty(type=NavMeshPolyRender)

    WindowManager.sz_ui_nav_view_bounds = BoolProperty(
        name="Display Grid Bounds", description="Display the navigation mesh map grid bounds on the 3D Viewport",
        default=False
    )

    WindowManager.sz_ui_nav_compute_edge_neighbors = BoolProperty(
        name="Compute Edge Neighbors", description="Compute polygon adjacency for the navmesh export. Should be checked when generating from collisions",
        default=False
    )


def unregister():
    del Object.sz_nav_cover_point
    del Object.sz_nav_link
    del Mesh.sz_navmesh_poly_access
    del Mesh.sz_navmesh_edge_access
    del Mesh.sz_navmesh_poly_render
    del WindowManager.sz_ui_nav_view_bounds