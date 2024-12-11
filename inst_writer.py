import math

from .scene_input import SceneInput
from .file_io import BinaryFileHelper
from .utils import Utils


class INSTWriter:
    def __init__(self, scene_input: SceneInput, filepath):
        self.scene_input = scene_input
        self.filepath = filepath
        self.utils = Utils(self.scene_input)

    def process_inst_elem(self, obj):
        rot = self.scene_input.get_rotation(obj)
        pr = self.scene_input.get_property_container(obj)
        scalex = self.scene_input.get_scale(obj)[0]
        scalez = self.scene_input.get_scale(obj)[2] #TODO: fix name (this is the vertical)
        scaley = self.scene_input.get_scale(obj)[1]
        origo = self.scene_input.get_position(obj)
        iblock = self.utils.get_block_number(obj)
        obj_name = pr["object"]
        flags = int(pr.get("flags", 0))
        self.export_elem(rot, scalex, scaley, scalez, origo, iblock, obj_name)

    def process_pkg_elem(self, obj):
        pass

    def export_elem(self, rot, scalex, scaley, scalez, origo, iblock, obj_name):
        file = self.file
        rx = rot[0]
        rz = rot[1]
        ry = rot[2]
        if ry == 0:
            ry = 0.0000000001  # else MM2 crashes for some reason
        ib = self.utils.get_real_block_number(self.blocks, iblock)
        file.write_uint16(ib + 1)
        file.write_uint16(flags)
        if (abs(1.0 - scalex) < 0.0001 and
            abs(1.0 - scaley) < 0.0001 and
            abs(1.0 - scalez) < 0.0001 and
            abs(rx) < 0.0002 and
                abs(rz) < 0.0002):
            file.write_byte(len(obj_name) + 129)
            file.write_zero_terminated_string(obj_name)
            file.write_float(math.cos(ry))
            file.write_float(-math.sin(ry))
        else:
            r11 = (math.cos(ry) * math.cos(rz) * scalex)
            r12 = (math.sin(rz) * scalex)
            r13 = (-math.sin(ry) * math.cos(rz) * scalex)
            r21 = (-math.cos(ry) * math.sin(rz) * scalez)
            r22 = (math.cos(rz) * scalez)
            r23 = (math.sin(ry) * math.sin(rz) * scalez)
            r31 = (math.sin(ry) * scaley)
            r32 = 0
            r33 = (math.cos(ry) * scaley)
            xaxis = [r11, r12, r13]
            yaxis = [r21, r22, r23]
            zaxis = [r31, r32, r33]
            file.write_byte(len(obj_name) + 1)
            file.write_zero_terminated_string(obj_name)
            file.write_float(xaxis[0])
            file.write_float(xaxis[1])
            file.write_float(xaxis[2])
            file.write_float(yaxis[0])
            file.write_float(yaxis[1])
            file.write_float(yaxis[2])
            file.write_float(zaxis[0])
            file.write_float(zaxis[1])
            file.write_float(zaxis[2])
        file.write_vec3(origo)

    def write(self):
        self.file = BinaryFileHelper(self.filepath.replace(".psdl", ".inst"), 'wb')
        self.blocks = self.utils.get_real_blocks()
        for obj in self.scene_input.get_object_list():
            typ = self.utils.get_object_type(obj)
            if typ == "INST":
                self.process_inst_elem(obj)
            elif typ == "PKG":
                self.process_pkg_elem(obj)
        self.file.close()
        print("INST exported!")
