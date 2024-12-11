import math
import os
import numpy as np

from .scene_input import SceneInput
from .file_io import BinaryFileHelper
from .utils import Utils


class PKGWriter:
    def __init__(self, scene_input: SceneInput, filepath):
        self.scene_input = scene_input
        self.filepath = filepath
        self.utils = Utils(self.scene_input)
        self.folder_path = filepath.replace(".psdl", "_geometry")
        self.basename = filepath.split('/')[-1].split('\\')[-1].replace('.psdl', '')
        self.groups = {}
        self.part_names = ["H", "M", "L", "VL", "BND"]

    def group_object(self, obj):
        groups = self.groups
        obj_name = self.scene_input.get_object_name(obj)
        pr = self.scene_input.get_property_container(obj)
        group_name = pr.get("pkg_name", None)
        if group_name:
            if not group_name in groups:
                groups[group_name] = {k: None for k in self.part_names}
        part = pr.get("lod_name")
        if part in self.part_names:
            groups[group_name][part] = obj
        else:
            name = pr.get("original_name", obj_name)
            print("Warning: object " + name + " has an invalid LOD: " + part)

    def process_group(self, group):
        for k in self.groups:
            group = self.groups[k]
            name = self.basename + "_" + k
            file = BinaryFileHelper(self.folder_path + "/" + name + ".pkg", 'wb')
            #todo
            file.close()
            if "BND" in group:
                #todo: bbnd/ter
                file = BinaryFileHelper(self.folder_path + "/" + name + ".bnd", 'w')
                #todo
                file.close()

    def write(self):
        # Create the folder for the pkgs
        if not os.path.exists(self.folder_path):
            os.mkdir(self.folder_path)
        
        # Each object labeled as PKG is one LOD or the collider,
        # so we have to group them
        for obj in self.scene_input.get_object_list():
            typ = self.utils.get_object_type(obj)
            if typ == "PKG":
                self.group_object(obj)
        
        # Each group becomes a pkg object (plus .bnd if present)
        for group in self.groups:
            self.process_group(group)
        
        print("PKGs exported!")
