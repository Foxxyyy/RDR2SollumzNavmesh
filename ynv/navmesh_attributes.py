from bpy.types import (
    Mesh,
)
import bmesh
from enum import Enum
from typing import NamedTuple
from collections.abc import Iterator
from dataclasses import dataclass


class NavMeshAttr(str, Enum):
    POLY_DATA_0 = ".navmesh.poly_data0"
    POLY_DATA_1 = ".navmesh.poly_data1"
    POLY_DATA_2 = ".navmesh.poly_data2"
    POLY_DATA_3 = ".navmesh.poly_data3"

    EDGE_AREA_ID = ".navmesh.edge_area_id"
    EDGE_POLY_ID = ".navmesh.edge_poly_id"
    EDGE_SPACE_AROUND_VERTEX = ".navmesh.edge_space_around_vertex"
    EDGE_FLAGS = ".navmesh.edge_flags"

    AUDIO_DATA = ".navmesh.audio_data"
    BUILD_ID = ".navmesh.audio_data"

    @property
    def type(self):
        match self:
            case NavMeshAttr.POLY_DATA_0 | NavMeshAttr.POLY_DATA_1 | NavMeshAttr.POLY_DATA_2 | NavMeshAttr.POLY_DATA_3:
                return "FLOAT"
            case NavMeshAttr.EDGE_AREA_ID | NavMeshAttr.EDGE_POLY_ID | NavMeshAttr.EDGE_SPACE_AROUND_VERTEX| NavMeshAttr.EDGE_FLAGS | NavMeshAttr.AUDIO_DATA | NavMeshAttr.BUILD_ID:
                return "INT"
            case _:
                assert False, f"Type not set for navmesh attribute '{self}'"

    @property
    def domain(self):
        match self:
            case NavMeshAttr.POLY_DATA_0 | NavMeshAttr.POLY_DATA_1 | NavMeshAttr.POLY_DATA_2 | NavMeshAttr.POLY_DATA_3 | NavMeshAttr.AUDIO_DATA:
                return "FACE"
            case NavMeshAttr.EDGE_AREA_ID | NavMeshAttr.EDGE_POLY_ID | NavMeshAttr.EDGE_SPACE_AROUND_VERTEX | NavMeshAttr.EDGE_FLAGS:
                return "CORNER"
            case _:
                assert False, f"Domain not set for navmesh attribute '{self}'"


def mesh_add_navmesh_attribute(mesh: Mesh, attr: NavMeshAttr):
    mesh.attributes.new(attr, attr.type, attr.domain)


def mesh_has_navmesh_attribute(mesh: Mesh, attr: NavMeshAttr) -> bool:
    return attr in mesh.attributes


@dataclass(slots=True)
class NavPolyAttributes:
    # Flags 1
    is_small: bool
    is_large: bool
    is_unk3: bool
    is_sheltered: bool
    is_unused10: bool
    is_unused20: bool
    is_switched_off_for_peds: bool
    is_water: bool

    # Flags 2
    is_danger_keep_away1: bool
    is_danger_keep_away2: bool
    is_sheltered2: bool
    is_unk12: bool
    is_near_car_node: bool
    is_interior: bool
    is_isolated: bool
    is_zero_area_stitch: bool
    is_network_spawn_candidate: bool
    is_road: bool
    is_unk19: bool
    is_lies_along_edge: bool

    # Flags 3
    is_unk21: bool
    is_unk22: bool
    is_unk23: bool
    is_muddy: bool
    is_sheltered3: bool
    is_vegetation: bool
    is_boardwalk: bool
    is_pavement: bool
    is_unk29: bool
    is_door: bool
    is_stairs: bool
    is_steep_slope: bool

    # Flags 4
    is_unk33: bool
    is_shelter4: bool
    is_unk35: bool
    is_unk36: bool

    is_water_depth: int # 0..7
    audio_property: int # 0..15

    @staticmethod
    def unpack(data0, data1, data2, data3, audio) -> "NavPolyAttributes":
        return NavPolyAttributes(
            # Flags1
            is_small=bool(data0 & 0x1),
            is_large=bool(data0 & 0x2),
            is_unk3=bool(data0 & 0x4),
            is_sheltered=bool(data0 & 0x8),
            is_unused10=bool(data0 & 0x10),
            is_unused20=bool(data0 & 0x20),
            is_switched_off_for_peds=bool(data0 & 0x40),
            is_water=bool(data0 & 0x80),

            # Flags2
            is_danger_keep_away1=bool(data1 & 0x1),
            is_danger_keep_away2=bool(data1 & 0x2),
            is_sheltered2=bool(data1 & 0x4),
            is_unk12=bool(data1 & 0x8),
            is_near_car_node=bool(data1 & 0x10),
            is_interior=bool(data1 & 0x20),
            is_isolated=bool(data1 & 0x40),
            is_zero_area_stitch=bool(data1 & 0x80),
            is_network_spawn_candidate=bool(data1 & 0x100),
            is_road=bool(data1 & 0x200),
            is_unk19=bool(data1 & 0x400),
            is_lies_along_edge=bool(data1 & 0x800),

            # Flags3
            is_unk21=bool(data2 & 0x1),
            is_unk22=bool(data2 & 0x2),
            is_unk23=bool(data2 & 0x4),
            is_muddy=bool(data2 & 0x8),
            is_sheltered3=bool(data2 & 0x10),
            is_vegetation=bool(data2 & 0x20),
            is_boardwalk=bool(data2 & 0x40),
            is_pavement=bool(data2 & 0x80),
            is_unk29=bool(data2 & 0x100),
            is_door=bool(data2 & 0x200),
            is_stairs=bool(data2 & 0x400),
            is_steep_slope=bool(data2 & 0x800),

            # Flags4
            is_unk33=bool(data3 & 0x1),
            is_shelter4=bool(data3 & 0x2),
            is_unk35=bool(data3 & 0x4),
            is_unk36=bool(data3 & 0x8),

            is_water_depth=(data3 >> 17) & 0x7,
            audio_property = audio
        )

    def pack(self) -> tuple[int, int, int, int, int]:
        flags1 = 0
        flags1 |= 0x1 if self.is_small else 0
        flags1 |= 0x2 if self.is_large else 0
        flags1 |= 0x4 if self.is_unk3 else 0
        flags1 |= 0x8 if self.is_sheltered else 0
        flags1 |= 0x10 if self.is_unused10 else 0
        flags1 |= 0x20 if self.is_unused20 else 0
        flags1 |= 0x40 if self.is_switched_off_for_peds else 0
        flags1 |= 0x80 if self.is_water else 0

        # Flags2
        flags2 = 0
        flags2 |= 0x1 if self.is_danger_keep_away1 else 0
        flags2 |= 0x2 if self.is_danger_keep_away2 else 0
        flags2 |= 0x4 if self.is_sheltered2 else 0
        flags2 |= 0x8 if self.is_unk12 else 0
        flags2 |= 0x10 if self.is_near_car_node else 0
        flags2 |= 0x20 if self.is_interior else 0
        flags2 |= 0x40 if self.is_isolated else 0
        flags2 |= 0x80 if self.is_zero_area_stitch else 0
        flags2 |= 0x100 if self.is_network_spawn_candidate else 0
        flags2 |= 0x200 if self.is_road else 0
        flags2 |= 0x400 if self.is_unk19 else 0
        flags2 |= 0x800 if self.is_lies_along_edge else 0

        # Flags3
        flags3 = 0
        flags3 |= 0x1 if self.is_unk21 else 0
        flags3 |= 0x2 if self.is_unk22 else 0
        flags3 |= 0x4 if self.is_unk23 else 0
        flags3 |= 0x8 if self.is_muddy else 0
        flags3 |= 0x10 if self.is_sheltered3 else 0
        flags3 |= 0x20 if self.is_vegetation else 0
        flags3 |= 0x40 if self.is_boardwalk else 0
        flags3 |= 0x80 if self.is_pavement else 0
        flags3 |= 0x100 if self.is_unk29 else 0
        flags3 |= 0x200 if self.is_door else 0
        flags3 |= 0x400 if self.is_stairs else 0
        flags3 |= 0x800 if self.is_steep_slope else 0

        # Flags4
        flags4 = 0
        flags4 |= 0x1 if self.is_unk33 else 0
        flags4 |= 0x2 if self.is_shelter4 else 0
        flags4 |= 0x4 if self.is_unk35 else 0
        flags4 |= 0x8 if self.is_unk36 else 0
        flags4 |= (self.is_water_depth & 0x7) << 17

        return flags1, flags2, flags3, flags4, self.audio_property


@dataclass(slots=True)
class NavEdgeAttributes:
    area_id: int
    poly_id: int
    space_around_vertex: int
    flags: int


def mesh_get_navmesh_poly_attributes(mesh: Mesh, poly_idx: int) -> NavPolyAttributes:
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
        bm.faces.ensure_lookup_table()

        data0_layer = bm.faces.layers.float[NavMeshAttr.POLY_DATA_0]
        data1_layer = bm.faces.layers.float[NavMeshAttr.POLY_DATA_1]
        data2_layer = bm.faces.layers.float[NavMeshAttr.POLY_DATA_2]
        data3_layer = bm.faces.layers.float[NavMeshAttr.POLY_DATA_3]
        audio_layer = bm.faces.layers.int[NavMeshAttr.AUDIO_DATA]

        data0 = 0 if data0_layer is None else bm.faces[poly_idx][data0_layer]
        data1 = 0 if data1_layer is None else bm.faces[poly_idx][data1_layer]
        data2 = 0 if data2_layer is None else bm.faces[poly_idx][data2_layer]
        data3 = 0 if data3_layer is None else bm.faces[poly_idx][data3_layer]
        audio = 0 if audio_layer is None else bm.faces[poly_idx][audio_layer]
    else:
        data0_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_0, None)
        data1_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_1, None)
        data2_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_2, None)
        data3_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_3, None)
        audio_attr = mesh.attributes.get(NavMeshAttr.AUDIO_DATA, None)

        data0 = 0 if data0_attr is None else data0_attr.data[poly_idx].value
        data1 = 0 if data1_attr is None else data1_attr.data[poly_idx].value
        data2 = 0 if data2_attr is None else data2_attr.data[poly_idx].value
        data3 = 0 if data3_attr is None else data3_attr.data[poly_idx].value
        audio = 0 if audio_attr is None else audio_attr.data[poly_idx].value

    return NavPolyAttributes.unpack(int(data0), int(data1), int(data2), int(data3), audio)


def mesh_iter_navmesh_all_poly_attributes(mesh: Mesh) -> Iterator[NavPolyAttributes]:
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
        try:
            data0_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_0)
            data1_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_1)
            data2_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_2)
            data3_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_3)
            audio_layer = bm.faces.layers.int.get(NavMeshAttr.AUDIO_DATA)

            for f in bm.faces:
                data0 = 0 if data0_layer is None else int(f[data0_layer])
                data1 = 0 if data1_layer is None else int(f[data1_layer])
                data2 = 0 if data2_layer is None else int(f[data2_layer])
                data3 = 0 if data3_layer is None else int(f[data3_layer])
                audio = 0 if audio_layer is None else f[audio_layer]
                yield NavPolyAttributes.unpack(data0, data1, data2, data3, audio)
        finally:
            bm.free()
    else:
        data0_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_0)
        data1_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_1)
        data2_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_2)
        data3_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_3)
        audio_attr = mesh.attributes.get(NavMeshAttr.AUDIO_DATA)

        for i in range(len(mesh.polygons)):
            data0 = 0 if data0_attr is None else int(data0_attr.data[i].value)
            data1 = 0 if data1_attr is None else int(data1_attr.data[i].value)
            data2 = 0 if data2_attr is None else int(data2_attr.data[i].value)
            data3 = 0 if data3_attr is None else int(data3_attr.data[i].value)
            audio = 0 if audio_attr is None else audio_attr.data[i].value
            yield NavPolyAttributes.unpack(data0, data1, data2, data3, audio)


def mesh_set_navmesh_poly_attributes(mesh: Mesh, poly_idx: int, poly_attrs: NavPolyAttributes):
    data0, data1, data2, data3, audio = poly_attrs.pack()

    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
        bm.faces.ensure_lookup_table()

        data0_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_0) or bm.faces.layers.float.new(NavMeshAttr.POLY_DATA_0)
        data1_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_1) or bm.faces.layers.float.new(NavMeshAttr.POLY_DATA_1)
        data2_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_2) or bm.faces.layers.float.new(NavMeshAttr.POLY_DATA_2)
        data3_layer = bm.faces.layers.float.get(NavMeshAttr.POLY_DATA_3) or bm.faces.layers.float.new(NavMeshAttr.POLY_DATA_3)
        audio_layer = bm.faces.layers.int.get(NavMeshAttr.AUDIO_DATA) or bm.faces.layers.int.new(NavMeshAttr.AUDIO_DATA)

        bm.faces[poly_idx][data0_layer] = float(data0)
        bm.faces[poly_idx][data1_layer] = float(data1)
        bm.faces[poly_idx][data2_layer] = float(data2)
        bm.faces[poly_idx][data3_layer] = float(data3)
        bm.faces[poly_idx][audio_layer] = audio
        bmesh.update_edit_mesh(mesh)
    else:
        data0_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_0)
        data1_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_1)
        data2_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_2)
        data3_attr = mesh.attributes.get(NavMeshAttr.POLY_DATA_3)
        audio_attr = mesh.attributes.get(NavMeshAttr.AUDIO_DATA)

        data0_attr.data[poly_idx].value = float(data0)
        data1_attr.data[poly_idx].value = float(data1)
        data2_attr.data[poly_idx].value = float(data2)
        data3_attr.data[poly_idx].value = float(data3)
        audio_attr.data[poly_idx].value = audio


def mesh_get_navmesh_edge_attributes(mesh: Mesh, edge_idx: int) -> NavEdgeAttributes:
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
        bm.edges.ensure_lookup_table()

        area_id_layer = bm.edges.layers.int[NavMeshAttr.EDGE_AREA_ID]
        poly_id__layer = bm.edges.layers.int[NavMeshAttr.EDGE_POLY_ID]
        space_around_layer = bm.edges.layers.int[NavMeshAttr.EDGE_SPACE_AROUND_VERTEX]
        flags_layer = bm.edges.layers.int[NavMeshAttr.EDGE_FLAGS]

        _area_id = 0 if area_id_layer is None else bm.edges[edge_idx][area_id_layer]
        _poly_id = 0 if poly_id__layer is None else bm.edges[edge_idx][poly_id__layer]
        _space = 0 if space_around_layer is None else bm.edges[edge_idx][space_around_layer]
        _flags = 0 if flags_layer is None else bm.edges[edge_idx][flags_layer]
    else:
        area_id_attr = mesh.attributes.get(NavMeshAttr.EDGE_AREA_ID, None)
        poly_id_attr = mesh.attributes.get(NavMeshAttr.EDGE_POLY_ID, None)
        space_attr = mesh.attributes.get(NavMeshAttr.EDGE_SPACE_AROUND_VERTEX, None)
        flags_attr = mesh.attributes.get(NavMeshAttr.EDGE_FLAGS, None)

        _area_id = 0 if area_id_attr is None else area_id_attr.data[edge_idx].value
        _poly_id = 0 if poly_id_attr is None else poly_id_attr.data[edge_idx].value
        _space = 0 if space_attr is None else space_attr.data[edge_idx].value
        _flags = 0 if flags_attr is None else flags_attr.data[edge_idx].value

    return NavEdgeAttributes(
        area_id=_area_id,
        poly_id=_poly_id,
        space_around_vertex=_space,
        flags=_flags
    )


def mesh_set_navmesh_edge_attributes(mesh: Mesh, edge_idx: int, edge_attrs: NavEdgeAttributes):
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
        bm.edges.ensure_lookup_table()

        area_id_layer = bm.edges.layers.int.get(NavMeshAttr.EDGE_AREA_ID)
        poly_id_layer = bm.edges.layers.int.get(NavMeshAttr.EDGE_POLY_ID)
        space_layer = bm.edges.layers.int.get(NavMeshAttr.EDGE_SPACE_AROUND_VERTEX)
        flags_layer = bm.edges.layers.int.get(NavMeshAttr.EDGE_FLAGS)

        if area_id_layer is not None:
            bm.edges[edge_idx][area_id_layer] = edge_attrs.area_id
        if poly_id_layer is not None:
            bm.edges[edge_idx][poly_id_layer] = edge_attrs.poly_id
        if space_layer is not None:
            bm.edges[edge_idx][space_layer] = edge_attrs.space_around_vertex
        if flags_layer is not None:
            bm.edges[edge_idx][flags_layer] = edge_attrs.flags
    else:
        area_id_attr = mesh.attributes.get(NavMeshAttr.EDGE_AREA_ID)
        poly_id_attr = mesh.attributes.get(NavMeshAttr.EDGE_POLY_ID)
        space_attr = mesh.attributes.get(NavMeshAttr.EDGE_SPACE_AROUND_VERTEX)
        flags_attr = mesh.attributes.get(NavMeshAttr.EDGE_FLAGS)

        if area_id_attr is not None:
            area_id_attr.data[edge_idx].value = edge_attrs.area_id
        if poly_id_attr is not None:
            poly_id_attr.data[edge_idx].value = edge_attrs.poly_id
        if space_attr is not None:
            space_attr.data[edge_idx].value = edge_attrs.space_around_vertex
        if flags_attr is not None:
            flags_attr.data[edge_idx].value = edge_attrs.flags