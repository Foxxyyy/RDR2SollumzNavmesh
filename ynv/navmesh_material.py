import bpy
from bpy.types import (
    Material
)
import numpy as np
from typing import NamedTuple
from ..ydr.shader_materials_v2 import organize_node_tree
from ..shared.shader_expr.builtins import (
    bitwise_and,
    flag_enabled,
    mix_color,
    truncf,
    vec,
    f2v,
    bsdf_diffuse,
    attribute,
    value,
    vec_value,
    roundf
)
from ..shared.shader_expr import expr, compile_to_material
from .navmesh_attributes import NavMeshAttr

NAVMESH_MATERIAL_NAME = ".sz.navmesh"


class FlagRenderInfo(NamedTuple):
    name: str
    data_index: int
    flag_value: int
    default_toggle: bool
    default_color: tuple[float, float, float]

    @property
    def toggle_name(self) -> str:
        return f"{self.name}__toggle"

    @property
    def color_name(self) -> str:
        return f"{self.name}__color"


class ValueRenderInfo(NamedTuple):
    name: str
    data_index: int
    start_bit: int
    end_bit: int
    default_toggle: bool
    default_color_min: tuple[float, float, float]
    default_color_max: tuple[float, float, float]

    @property
    def toggle_name(self) -> str:
        return f"{self.name}__toggle"

    @property
    def color_min_name(self) -> str:
        return f"{self.name}__color_min"

    @property
    def color_max_name(self) -> str:
        return f"{self.name}__color_max"


ALL_FLAGS = tuple(FlagRenderInfo(*args) for args in (
    ("DangerKeepAway1",            1, 0x100, False, (0.25, 0.0, 0.0)),
    ("DangerKeepAway2",            1, 0x200, False, (0.25, 0.0, 0.0)),
    ("Pavement",                   2, 0x8000000, False,  (0.0, 0.25, 0.0)),
    ("Door",                       2, 0x20000000, False, (0.5, 0.0, 0.5)),
    ("Stairs",                     2, 0x40000000, False, (0.5, 0.25, 0.0)),
    ("Water",                      0, 0x80, False, (0.0, 0.0, 0.15)),
    ("Muddy",                      2, 0x800000, False, (0.2, 0.15, 0.0)),
    ("Interior",                   1, 0x2000, False, (0.0, 0.0, 0.1)),
    ("Road",                       1, 0x20000, False, (0.0, 0.0, 0.1)),
))


def get_int_attribute_array(mesh, attr_name):
    if attr_name not in mesh.attributes:
        return None
    attr = mesh.attributes[attr_name]
    return np.array([elem.value for elem in attr.data], dtype=np.int32)

def navmesh_material_shader_expr() -> expr.ShaderExpr:
    data0 = roundf(attribute(NavMeshAttr.POLY_DATA_0).fac)
    data1 = roundf(attribute(NavMeshAttr.POLY_DATA_1).fac)
    data2 = roundf(attribute(NavMeshAttr.POLY_DATA_2).fac)
    data3 = roundf(attribute(NavMeshAttr.POLY_DATA_3).fac)
    data = (data0, data1, data2, data3)

    slice_shifts = [0.0, 8.0, 20.0, 32.0]
    eps = 0.00001
    color = vec(0.05, 0.05, 0.05)

    for flag_info in ALL_FLAGS:
        flag_toggle = value(flag_info.toggle_name, default_value=1.0 if flag_info.default_toggle else 0.0)
        flag_color = vec_value(flag_info.color_name, default_value=flag_info.default_color)
        data_shifted = data[flag_info.data_index] * (2**slice_shifts[flag_info.data_index])
        flag = ((data_shifted / (flag_info.flag_value - eps)) % 2.0) > (1.0 - eps)
        color += f2v(flag * flag_toggle) * flag_color

    return bsdf_diffuse(
        color=color,
    )


def get_navmesh_material() -> Material:
    mat = bpy.data.materials.get(NAVMESH_MATERIAL_NAME, None)
    if mat is not None:
        return mat

    mat = compile_to_material(NAVMESH_MATERIAL_NAME, navmesh_material_shader_expr())
    organize_node_tree(mat.node_tree)

    return mat