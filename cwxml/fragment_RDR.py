from abc import ABC as AbstractClass
from dataclasses import dataclass
from mathutils import Matrix, Vector
from xml.etree import ElementTree as ET

from ..sollumz_properties import SollumzGame, set_import_export_current_game
from .element import (
    AttributeProperty,
    ElementTree,
    ElementProperty,
    FlagsProperty,
    ListProperty,
    MatrixProperty,
    QuaternionProperty,
    Vector4Property,
    TextProperty,
    ValueProperty,
    VectorProperty
)
from .drawable import Drawable, DrawableMatrices
from .bound import RDRBoundFile


class RDRFragDrawable(Drawable):
    def __init__(self, tag_name: str = "Drawable"):
        set_import_export_current_game(SollumzGame.RDR)
        super().__init__(tag_name)

        # Redefine these properties as they have different tag names in RDR2 XML compared to GTA5
        self.name = TextProperty("FragName", "")
        self.frag_bound_matrix = MatrixProperty("FragMatrix")
        self.frag_extra_bound_matrices = DrawableMatrices("ExtraMatrices")


class FragDrawableList(ListProperty):
    list_type = RDRFragDrawable
    tag_name = None
    item_tag_name = "Item"
    allow_none_items = True

    def __init__(self, tag_name: str):
        super().__init__(tag_name)

    @classmethod
    def from_xml(cls, element: ET.Element):
        new = cls()

        children = element.findall(cls.item_tag_name)

        for child in children:
            if "type" in child.attrib and child.get("type") == "null":
                new.value.append(None)
            else:
                new.value.append(RDRFragDrawable.from_xml(child))

        return new


class FragDrawableList1(FragDrawableList):
    tag_name = "Drawables1"

    def __init__(self):
        super().__init__(FragDrawableList1.tag_name)


class FragDrawableList2(FragDrawableList):
    tag_name = "Drawables2"

    def __init__(self):
        super().__init__(FragDrawableList2.tag_name)


class RDRBoneTransform(ElementProperty):
    tag_name = "Item"
    value_types = (Matrix)

    def __init__(self, tag_name: str, value=None):
        super().__init__(tag_name, value or Matrix())

    @staticmethod
    def from_xml(element: ET.Element):
        s_mtx = element.text.strip().split(" ")
        s_mtx = [s for s in s_mtx if s]  # removes empty strings
        m = Matrix()
        r_idx = 0
        item_idx = 0
        for r_idx in range(0, 3):
            for c_idx in range(0, 4):
                m[r_idx][c_idx] = float(s_mtx[item_idx])
                item_idx += 1

        return MatrixProperty(element.tag, m)

    def to_xml(self):
        if self.value is None:
            return

        matrix: Matrix = self.value

        lines = [" ".join([str(x) for x in row]) for row in matrix]

        element = ET.Element(self.tag_name)
        element.text = " ".join(lines)

        return element


class BoneTransformsList(ListProperty):
    list_type = RDRBoneTransform
    tag_name = "BoneTransforms"

    def __init__(self, tag_name=None):
        super().__init__(tag_name or BoneTransformsList.tag_name)


class RDRArchetype(ElementTree):
    tag_name = "Archetype"

    def __init__(self):
        super().__init__()
        self.name = TextProperty("Name")
        self.type_flags = FlagsProperty("TypeFlags")
        self.mass = QuaternionProperty("Mass")
        self.mass_inv = QuaternionProperty("MassInv")
        self.gravity_factor = ValueProperty("GravityFactor")
        self.max_speed = ValueProperty("MaxSpeed")
        self.max_ang_speed = ValueProperty("MaxAngSpeed")
        self.buoyancy_factor = ValueProperty("BuoyancyFactor")
        self.linear_c = Vector4Property("LinearC")
        self.linear_v = Vector4Property("LinearV")
        self.linear_v2 = Vector4Property("LinearV2")
        self.angular_c = Vector4Property("AngularC")
        self.angular_v = Vector4Property("AngularV")
        self.angular_v2 = Vector4Property("AngularV2")


class RDRPhysicsGroup(ElementTree):
    tag_name = "Item"

    def __init__(self):
        super().__init__()
        self.name = TextProperty("Name")
        self.flags = ValueProperty("Unknown00", 0)
        self.unknown_01 = ValueProperty("Unknown01", 0)
        self.parent_index = ValueProperty("ParentIndex")
        self.child_index = ValueProperty("ChildIndex")
        self.child_count = ValueProperty("ChildCount")
        self.bone_id = ValueProperty("BoneId")
        self.unknown_10 = ValueProperty("Unknown10", 0)
        self.strength = ValueProperty("Strength")
        self.force_transmission_scale_up = ValueProperty("ForceTransmissionScaleUp")
        self.force_transmission_scale_down = ValueProperty("ForceTransmissionScaleDown")
        self.joint_stiffness = ValueProperty("JointStiffness")
        self.min_soft_angle_1 = ValueProperty("MinSoftAngle1")
        self.max_soft_angle_1 = ValueProperty("MaxSoftAngle1")
        self.max_soft_angle_2 = ValueProperty("MaxSoftAngle2")
        self.max_soft_angle_3 = ValueProperty("MaxSoftAngle3")
        self.rotation_speed = ValueProperty("RotationSpeed")
        self.rotation_strength = ValueProperty("RotationStrength")
        self.restoring_strength = ValueProperty("RestoringStrength")
        self.restoring_max_torque = ValueProperty("RestoringMaxTorque")
        self.unknown_2A = ValueProperty("Unknown2A")
        self.unknown_2C = ValueProperty("Unknown2C")
        self.unknown_2E = ValueProperty("Unknown2E")
        self.unknown_30 = ValueProperty("Unknown30")
        self.min_damage_force = ValueProperty("MinDamageForce")
        self.damage_health = ValueProperty("DamageHealth")
        self.weapon_health = ValueProperty("WeaponHealth")
        self.weapon_scale = ValueProperty("WeaponScale")
        self.vehicle_scale = ValueProperty("VehicleScale")
        self.ped_scale = ValueProperty("PedScale")
        self.ragdoll_scale = ValueProperty("RagdollScale")
        self.explosion_scale = ValueProperty("ExplosionScale")
        self.object_scale = ValueProperty("ObjectScale")
        self.ped_inv_mass_scale = ValueProperty("PedInvMassScale")
        self.melee_scale = ValueProperty("MeleeScale")
        self.unknown_48 = ValueProperty("Unknown48")
        self.unknown_4A = ValueProperty("Unknown4A")
        self.unknown_4C = ValueProperty("Unknown4C")
        self.glass_window_index = ValueProperty("Unknown4E", 0)


class RDRGroupsList(ListProperty):
    list_type = RDRPhysicsGroup
    tag_name = "Groups"


class ChildrenFloatList(ElementProperty):
    value_types = (list)
    tag_name = None

    def __init__(self, tag_name: str, value=None):
        super().__init__(tag_name, value or [])

    @staticmethod
    def from_xml(element: ET.Element):
        new = ChildrenFloatList(element.tag, [])
        text = element.text.strip().split("\n")
        if len(text) > 0:
            for line in text:
                new.value.extend(float(x) for x in line.strip().split(" "))
        return new

    def to_xml(self):
        element = ET.Element(self.tag_name)
        element.text = "\n"

        if len(self.value) == 0:
            return None

        for v in self.value:
            element.text += (v + "\n")

        return element


class ChildrenIntList(ElementProperty):
    value_types = (list)
    tag_name = None

    def __init__(self, tag_name: str, value=None):
        super().__init__(tag_name, value or [])

    @staticmethod
    def from_xml(element: ET.Element):
        new = ChildrenFloatList(element.tag, [])
        text = element.text.strip().split("\n")
        if len(text) > 0:
            for line in text:
                new.value.extend(int(x) for x in line.strip().split(" "))
        return new

    def to_xml(self):
        element = ET.Element(self.tag_name)
        element.text = "\n"

        if len(self.value) == 0:
            return None

        for v in self.value:
            element.text += (v + "\n")

        return element


class ChildrenVecItem(ElementTree):
    tag_name = "Item"

    def __init__(self) -> None:
        super().__init__()
        self.x = AttributeProperty("x", 0)
        self.y = AttributeProperty("y", 0)
        self.z = AttributeProperty("z", 0)
        self.w = AttributeProperty("w", 0)


class ChildrenVecList(ListProperty):
    list_type = ChildrenVecItem
    tag_name = None

    def __init__(self, tag_name: str, value=None):
        super().__init__(tag_name, value or [])


class PaneModel(ElementTree):
    tag_name = "Item"

    def __init__(self) -> None:
        super().__init__()
        self.projection_matrix = MatrixProperty("Projection")
        # self.vertex_layout =
        self.vertex_count = ValueProperty("VertexCount")
        self.unknown_180 = ValueProperty("Unknown180")
        self.unknown_184 = ValueProperty("Unknown184", 2)
        self.frag_index = ValueProperty("FragIndex")
        self.thickness = ValueProperty("Thickness")
        self.tangent = VectorProperty("Tangent")
        self.unknown_198 = ValueProperty("Unknown198")

    @property
    def glass_type(self) -> int:
        # This field is actually a byte at offset 0x186, but CX does not export it to XML since apparently it's always 0 in the files...
        return 0


class PaneModelList(ListProperty):
    list_type = PaneModel
    tag_name = "PaneModelInfos"


@dataclass
class PhysicsChild:
    """Class that provides access to fragments children similar to GTA5's PhysicsChild.
    In the RDR2 XML they are stored in SoA format, while in GTA5 they were in AoS.
    """
    group_index: int
    bone_tag: int
    pristine_mass: float
    damaged_mass: float
    unk_float: float
    damaged_inertia_tensor: Vector
    inertia_tensor: Vector
    drawable: RDRFragDrawable
    damaged_drawable: RDRFragDrawable


class RDRPhysicsLOD(ElementTree):
    tag_name = "PhysicsLOD1"

    def __init__(self, tag_name="PhysicsLOD1"):
        super().__init__()
        self.tag_name = tag_name
        self.min_move_force = ValueProperty("MinMoveForce")
        self.original_root_cg_offset = Vector4Property("OriginalRootCgOffset")
        self.archetype = RDRArchetype()
        self.groups = RDRGroupsList()
        self.children_unk_floats = ChildrenFloatList("ChildrenUnkFloats")
        self.children_pristine_mass = ChildrenFloatList("ChildrenPristineMass")
        self.children_damaged_mass = ChildrenFloatList("ChildrenDamagedMass")
        self.children_group_indices = ChildrenIntList("ChildrenGroupIndices")
        self.children_damaged_inertia_tensors = ChildrenVecList("ChildrenUnkVecs")
        self.children_inertia_tensors = ChildrenVecList("ChildrenInertiaTensors")
        self.drawables = FragDrawableList1()
        self.damaged_drawables = FragDrawableList2()
        self.bounds = RDRBoundFile()
        self._children = None

    @property
    def children(self) -> list[PhysicsChild]:
        """Access children data as AoS."""
        if self._children is None:
            groups = self.groups
            child_to_group_index = self.children_group_indices
            num_children = len(child_to_group_index)
            pristine_masses = self.children_pristine_mass
            damaged_masses = self.children_pristine_mass
            unk_floats = self.children_unk_floats
            inertia_tensors = self.children_inertia_tensors
            damaged_inertia_tensors = self.children_damaged_inertia_tensors
            drawables = self.drawables
            damaged_drawables = self.damaged_drawables

            children = []
            for ci in range(num_children):
                gi = child_to_group_index[ci]
                # TODO: is this correct? in GTA5 the bone was defined in the child not the group
                #       and children in the same group could be attached to different bones
                bone_tag = groups[gi].bone_id
                c = PhysicsChild(
                    gi,
                    bone_tag,
                    pristine_masses[ci],
                    damaged_masses[ci],
                    unk_floats[ci],
                    damaged_inertia_tensors[ci],
                    inertia_tensors[ci],
                    drawables[ci],
                    damaged_drawables[ci]
                )
                children.append(c)

            self._children = children

        return self._children


class RDRPhysicsLODGroup(ElementTree):
    tag_name = "PhysicsLODGroup"

    def __init__(self):
        super().__init__()
        self.lod1 = RDRPhysicsLOD("PhysicsLOD1")
        self.lod2 = RDRPhysicsLOD("PhysicsLOD2")
        self.lod3 = RDRPhysicsLOD("PhysicsLOD3")


class RDRFragment(ElementTree, AbstractClass):
    tag_name = "RDR2Fragment"

    def __init__(self):
        super().__init__()
        self.version = AttributeProperty("version", 0)
        self.name = TextProperty("Name")
        self.bounding_sphere_center = VectorProperty("BoundingSphereCenter")
        self.bounding_sphere_radius = ValueProperty("BoundingSphereRadius")
        self.nm_asset_id = ValueProperty("NmAssetID")
        self.break_and_damage_flags = ValueProperty("BreakAndDamageFlags")
        self.unknown_84h = ValueProperty("Unknown_84h")
        self.drawable = RDRFragDrawable()
        self.bones_transforms = BoneTransformsList()
        self.glass_windows = PaneModelList()
        self.physics = RDRPhysicsLODGroup()

    def get_lods_by_id(self):
        return {1: self.physics.lod1, 2: self.physics.lod2, 3: self.physics.lod3}
