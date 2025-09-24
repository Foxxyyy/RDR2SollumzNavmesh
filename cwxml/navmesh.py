from .element import (
    ElementTree,
    ListProperty,
    TextProperty,
    ValueProperty,
    VectorProperty
)
from xml.etree import ElementTree as ET
from mathutils import Vector


class YNV:

    file_extension = ".ynv.xml"

    @staticmethod
    def from_xml_file(filepath):
        return Navmesh.from_xml_file(filepath)

    @staticmethod
    def write_xml(nav, filepath):
        return nav.write_xml(filepath)


class NavPoint(ElementTree):
    tag_name = "Item"

    def __init__(self):
        super().__init__()
        self.type = ValueProperty("Type")
        self.angle = ValueProperty("Angle")
        self.position = VectorProperty("Position")

    def to_xml(self):
        return super().to_xml()


class NavPointList(ListProperty):
    list_type = NavPoint
    tag_name = "Points"


class NavLink(ElementTree):
    tag_name = "Item"

    def __init__(self):
        super().__init__()
        self.type = ValueProperty("Type")
        self.angle = ValueProperty("Angle")
        self.area_from = ValueProperty("AreaID1")
        self.area_to = ValueProperty("AreaID2")
        self.poly_from = ValueProperty("PolyID1")
        self.poly_to = ValueProperty("PolyID2")
        self.position_from = VectorProperty("Position1")
        self.position_to = VectorProperty("Position2")

    def to_xml(self):
        return super().to_xml()


class NavLinkList(ListProperty):
    list_type = NavLink
    tag_name = "SpecialLinks"


class NavPolygonVertices(ListProperty):
    list_type = Vector
    tag_name = "Vertices"

    def __init__(self, tag_name=None, value=None):
        super().__init__(tag_name=tag_name, value=value)

    @classmethod
    def from_xml(cls, element: ET.Element):
        new = cls()
        verts = []

        for item in element.findall("Item"):
            x = float(item.get("x", 0))
            y = float(item.get("y", 0))
            z = float(item.get("z", 0))
            verts.append(Vector((x, y, z)))
        new.value = verts
        return new
    
    def to_xml(self):
        element = ET.Element(self.tag_name)
        if self.value:
            for v in self.value:
                item = ET.Element("Item")
                item.set("x", str(v.x))
                item.set("y", str(v.y))
                item.set("z", str(v.z))
                element.append(item)
        return element


class NavPolygonEdges(ListProperty):
    list_type = tuple  # each edge is a tuple of 4 ints: (areaID, polyID, spaceAroundVertex, flags)
    tag_name = "Edges"

    def __init__(self, tag_name=None, value=None):
        super().__init__(tag_name=tag_name, value=value)

    @classmethod
    def from_xml(cls, element: ET.Element):
        new = cls()
        edges = []

        for item in element.findall("Item"):
            text = item.text

            if not text or not text.strip():
                continue
            
            parts = text.split(";")
            while len(parts) < 4:
                parts += ["0"] * (4 - len(parts))

            edge_tuple = []
            for i, p in enumerate(parts):
                p = p.strip()
                if i < 3: # first 3 are integers
                    try:
                        edge_tuple.append(int(p))
                    except ValueError:
                        edge_tuple.append(0)
                else: # last is a flag
                    edge_tuple.append(Navmesh.edge_flags.get(p, 0))

            edges.append(tuple(edge_tuple))
        new.value = edges
        return new

    def to_xml(self):
        element = ET.Element(self.tag_name)
        if self.value:
            lines = self.value.splitlines() if isinstance(self.value, str) else self.value
            for line in lines:
                parts = [p.strip() for p in line.split(";")]
                if len(parts) < 4:
                    continue

                a, b, c = int(float(parts[0])), int(float(parts[1])), int(float(parts[2]))
                p3 = parts[3] # CodeX exports the edge flag as a string or integer...
                try:
                    flag_val = int(p3)
                except ValueError:
                    flag_val = Navmesh.edge_flags.get(p3, 0)

                flag_str = next((k for k, v in Navmesh.edge_flags.items() if v == flag_val), str(flag_val))

                item = ET.Element("Item")
                item.text = f"{a}; {b}; {c}; {flag_str}"
                element.append(item)
        return element


class NavPolygon(ElementTree):
    tag_name = "Item"

    def __init__(self, is_terrain_poly=False):
        super().__init__()
        self.flags = TextProperty("Flags")
        self.audio_data = ValueProperty("Audio")
        self.water_depth = ValueProperty("WaterDepth")
        self.centroid_x = ValueProperty("CentroidX")
        self.centroid_y = ValueProperty("CentroidY")
        self.vertices = NavPolygonVertices("Vertices")
        self.edges = NavPolygonEdges("Edges")
        self.is_terrain_poly = is_terrain_poly

    def to_xml(self):
        return super().to_xml()


class NavPolygonList(ListProperty):
    list_type = NavPolygon
    tag_name = "Polygons"


class Navmesh(ElementTree):
    tag_name = "RDR2Navmesh"

    polygon_flags = {
        "Small": 1,
        "Large": 2,
        "Unk3": 4,
        "Sheltered": 8,
        "Unused10": 0x10,
        "Unused20": 0x20,
        "SwitchedOffForPeds": 0x40,
        "Water": 0x80,
        "DangerKeepAway1": 0x100,
        "DangerKeepAway2": 0x200,
        "Sheltered2": 0x400,
        "Unk12": 0x800,
        "NearCarNode": 0x1000,
        "Interior": 0x2000,
        "Isolated": 0x4000,
        "ZeroAreaStitchPolyDLC": 0x8000,
        "NetworkSpawnCandidate": 0x10000,
        "Road": 0x20000,
        "Unk19": 0x40000,
        "LiesAlongEdgeOfMesh": 0x80000,
        "Unk21": 0x100000,
        "Unk22": 0x200000,
        "Unk23": 0x400000,
        "Muddy": 0x800000,
        "Sheltered3": 0x1000000,
        "Vegetation": 0x2000000,
        "Boardwalk": 0x4000000,
        "Pavement": 0x8000000,
        "Unk29": 0x10000000,
        "Door": 0x20000000,
        "Stairs": 0x40000000,
        "SteepSlope": 0x80000000,
        "Unk33": 0x100000000,
        "Shelter4": 0x200000000,
        "Unk35": 0x400000000,
        "Unk36": 0x800000000,
    }

    edge_flags = {
        "AdjacencyDisabled": 1,
        "EdgeProvidesCover": 2,
        "HighDropOverEdge": 4,
        "ExternalEdge": 8,
        "Unknown16": 16,
    }

    def flags_to_names(flag0: int, flag1: int, flag2: int, flag3: int) -> str:
        result = []

        # flag0
        if flag0 & 0x1: result.append("Small")
        if flag0 & 0x2: result.append("Large")
        if flag0 & 0x4: result.append("Unk3")
        if flag0 & 0x8: result.append("Sheltered")
        if flag0 & 0x10: result.append("Unused10")
        if flag0 & 0x20: result.append("Unused20")
        if flag0 & 0x40: result.append("SwitchedOffForPeds")
        if flag0 & 0x80: result.append("Water")

        # flag1
        if flag1 & 0x1: result.append("DangerKeepAway1")
        if flag1 & 0x2: result.append("DangerKeepAway2")
        if flag1 & 0x4: result.append("Sheltered2")
        if flag1 & 0x8: result.append("Unk12")
        if flag1 & 0x10: result.append("NearCarNode")
        if flag1 & 0x20: result.append("Interior")
        if flag1 & 0x40: result.append("Isolated")
        if flag1 & 0x80: result.append("ZeroAreaStitchPolyDLC")
        if flag1 & 0x100: result.append("NetworkSpawnCandidate")
        if flag1 & 0x200: result.append("Road")
        if flag1 & 0x400: result.append("Unk19")
        if flag1 & 0x800: result.append("LiesAlongEdgeOfMesh")

        # flag2
        if flag2 & 0x1: result.append("Unk21")
        if flag2 & 0x2: result.append("Unk22")
        if flag2 & 0x4: result.append("Unk23")
        if flag2 & 0x8: result.append("Muddy")
        if flag2 & 0x10: result.append("Sheltered3")
        if flag2 & 0x20: result.append("Vegetation")
        if flag2 & 0x40: result.append("Boardwalk")
        if flag2 & 0x80: result.append("Pavement")
        if flag2 & 0x100: result.append("Unk29")
        if flag2 & 0x200: result.append("Door")
        if flag2 & 0x400: result.append("Stairs")
        if flag2 & 0x800: result.append("SteepSlope")

        # flag3
        if flag3 & 0x1: result.append("Unk33")
        if flag3 & 0x2: result.append("Shelter4")
        if flag3 & 0x4: result.append("Unk35")
        if flag3 & 0x8: result.append("Unk36")

        return ", ".join(result) if result else "None"

    def __init__(self):
        super().__init__()
        self.content_flags = TextProperty("ContentFlags")
        self.area_id = ValueProperty("AreaID")
        self.build_id = ValueProperty("BuildID")
        self.bb_min = VectorProperty("AABBMin")
        self.bb_max = VectorProperty("AABBMax")
        self.bb_size = VectorProperty("Extents")
        self.adj_area_ids = TextProperty("AdjAreaIDs")
        self.polygons = NavPolygonList()
        self.links = NavLinkList()
        self.points = NavPointList()

    def to_xml(self):
        return super().to_xml()
