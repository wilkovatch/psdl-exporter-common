# Info here: https://github.com/Dummiesman/angel-file-formats/blob/master/Midtown%20Madness%202/BAI.md

import numpy as np

from .scene_input import SceneInput
from .file_io import BinaryFileHelper
from .utils import Utils
import math


class BAIWriter:
    def __init__(self, scene_input: SceneInput, filepath, accurate_culling):
        self.scene_input = scene_input
        self.filepath = filepath
        self.accurate_culling = accurate_culling
        self.utils = Utils(self.scene_input)

    def prop(self, obj):
        return self.scene_input.get_property_container(obj)

    def verts_distance(self, v1, v2):
        return np.linalg.norm(np.subtract(v1, v2))

    def vector_projection(self, oA, oB, oC):
        A = np.subtract(oC, oA)
        B = np.subtract(oB, oA)
        NB = self.utils.get_normal(B)
        C = np.multiply(np.dot(A, NB), NB)
        return np.add(C, oA)

    def rotate_point(self, point_to_rotate, center_point, angle):
        pR = point_to_rotate
        pC = center_point
        theta = np.deg2rad(angle)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        X = (cos_theta * (pR[0] - pC[0])) - (sin_theta * (pR[1] - pC[1])) + pC[0]
        Y = (sin_theta * (pR[0] - pC[0])) + (cos_theta * (pR[1] - pC[1])) + pC[1]
        return [X, Y, pR[2]]

    def get_which_sides_have_sidewawlks(self, value):
        if ":" in value:
            parts = value.split(":")
            return [int(parts[0]), int(parts[1])]
        else:
            res = int(value)
            return [res, res]

    def get_intersection_center(self, IS):
        vertices_sum = 0
        verts_num = self.scene_input.get_vertices_num(IS)
        for i in range(verts_num):
            vertices_sum = np.add(vertices_sum, self.scene_input.get_vertex(IS, i))
        return np.divide(vertices_sum, verts_num)

    def write_road_end(self, road, end_index):
        road_properties = self.prop(road)
        file = self.file
        road_intersections = [road_properties["start_intersection"], road_properties["end_intersection"]]
        road_rules = [road_properties["start_rule"], road_properties["end_rule"]]
        index_in_intersection = -1
        intersection_index = -1
        this_road_id = road_properties["id"]
        block_id = road_intersections[end_index]
        if block_id == "0":
            # disconnected road
            road_rule = "2"
            index_in_intersection = 3452816845  # cdcdcdcd
        else:
            road_rule = road_rules[end_index]

            # Find the intersection
            intersection = None
            for i in range(len(self.intersections_finished)):
                temp_intersection = self.intersections_finished[i]
                if block_id == self.prop(temp_intersection)["block"]:
                    intersection = temp_intersection
                    intersection_index = i
            intersection_roads = self.prop(intersection)["roads"].split(";")

            # Find the index of this road in the intersection
            for i in range(len(intersection_roads)):
                if intersection_roads[i] == this_road_id:
                    index_in_intersection = i

        # Write the road end
        file.write_uint32(intersection_index)
        file.write_uint16(52685)  # cdcd
        file.write_uint32(int(road_rule))
        file.write_uint32(index_in_intersection)

        if road_rules[end_index] == "1":
            # Automatic traffic lights placement
            # They are placed on the opposite road (as in MM2)
            # (it actually uses the road with index+2, so only
            # for intersections with 4 roads it's the opposite.
            # It also excludes roads without traffic from the count.)

            # Find the opposite road
            this_road_ok = False
            opposite_road_index = index_in_intersection + 2
            if opposite_road_index >= len(intersection_roads):
                # Avoid overflow
                opposite_road_index -= len(intersection_roads)
            while not this_road_ok:
                # Check the current road
                i = intersection_roads[opposite_road_index]
                road_2 = None
                for temp_road in self.roads_finished:
                    if i == self.prop(temp_road)["id"]:
                        road_2 = temp_road
                road_2_properties = self.prop(road_2)
                n_lanes_A = road_2_properties["left_lanes"]
                n_lanes_B = road_2_properties["right_lanes"]
                sides_have_sidewalks = self.get_which_sides_have_sidewawlks(road_2_properties["has_sidewalks"])
                traf_type = int(road_2_properties["traffic_type"])
                if traf_type == 3 and (n_lanes_A == "0" or n_lanes_B == "0"):
                    # Skip roads without traffic
                    opposite_road_index += 1
                    if opposite_road_index >= len(intersection_roads):
                        opposite_road_index -= len(intersection_roads)
                else:
                    # Opposite road found
                    this_road_ok = True

            # Place the traffic light
            light_dir = 1 if road_2_properties["end_intersection"] == block_id else 0
            road_2_vps = int(road_2_properties["vertices_per_section"])
            vo1 = [0.0, 0.0, 0.0]
            vo2 = [0.0, 0.0, 0.0]
            if light_dir == 1:
                vo1 = self.scene_input.get_vertex(road_2, -road_2_vps + sides_have_sidewalks[1])
                vo2 = self.scene_input.get_vertex(road_2, -road_2_vps)
            else:
                vo1 = self.scene_input.get_vertex(road_2, road_2_vps - sides_have_sidewalks[0] - 1)
                vo2 = self.scene_input.get_vertex(road_2, road_2_vps - 1)
            vo = np.divide(np.add(np.multiply(vo1, 3), vo2), 4)
            file.write_vec3(vo)
            road_1_vps = int(road_properties["vertices_per_section"])
            oA = [0.0, 0.0, 0.0]
            oB = [0.0, 0.0, 0.0]
            if end_index == 0:
                oA = self.scene_input.get_vertex(road, road_1_vps - 1)
                oB = self.scene_input.get_vertex(road, 0)
            elif end_index == 1:
                oA = self.scene_input.get_vertex(road, -road_1_vps)
                oB = self.scene_input.get_vertex(road, -1)
            vd0 = self.vector_projection(oA, oB, vo)
            vd = self.rotate_point(vd0, vo, 90.0)
            file.write_vec3(vd)
        else:
            # Empty data for (unused) traffic lights
            for i in range(6):
                file.write_float(0.0)

    def write_road_data(self, rd, is_right_side):
        rd_pr = self.scene_input.get_property_container(rd)
        file = self.file
        vps = int(rd_pr["vertices_per_section"])
        this_side_has_sidewalks = self.get_which_sides_have_sidewawlks(rd_pr["has_sidewalks"])[1 if is_right_side else 0] > 0
        traffic_type = int(rd_pr["traffic_type"])
        n_sections = self.scene_input.get_vertices_num(rd) // vps
        n_lanes = int(rd_pr["left_lanes"] if is_right_side else rd_pr["right_lanes"])
        if n_lanes == 0:
            if traffic_type == 0 or traffic_type == 1:
                traffic_type = 1
            else:
                traffic_type = 3
        n_lanes_other_side = int(rd_pr["right_lanes"] if is_right_side else rd_pr["left_lanes"])
        file.write_uint16(n_lanes)
        file.write_uint16(0)
        file.write_uint16(0)
        file.write_uint16(1)
        file.write_uint16(traffic_type)
        lanes = []
        n_sections_range = range(0, n_sections)
        if is_right_side:
            n_sections_range = list(reversed(n_sections_range))
        for i in range(n_lanes):
            lane = []
            for i2 in n_sections_range:
                v1 = [0.0, 0.0, 0.0]
                v2 = [0.0, 0.0, 0.0]
                if this_side_has_sidewalks:
                    if vps == 6:
                        x = -2 if is_right_side else 0
                        v1 = self.scene_input.get_vertex(rd, i2 * vps + 3 + x)
                        v2 = self.scene_input.get_vertex(rd, i2 * vps + 4 + x)
                    else:
                        if is_right_side:
                            if n_lanes_other_side == 0:
                                v2 = self.scene_input.get_vertex(rd, i2 * vps + 2)
                            else:
                                v2 = np.divide(np.add(self.scene_input.get_vertex(rd, i2 * vps + 1), self.scene_input.get_vertex(rd, i2 * vps + 2)), 2)
                            v1 = self.scene_input.get_vertex(rd, i2 * vps + 1)
                        else:
                            if n_lanes_other_side == 0:
                                v1 = self.scene_input.get_vertex(rd, i2 * vps + 1)
                            else:
                                v1 = np.divide(np.add(self.scene_input.get_vertex(rd, i2 * vps + 1), self.scene_input.get_vertex(rd, i2 * vps + 2)), 2)
                            v2 = self.scene_input.get_vertex(rd, i2 * vps + 2)
                else:
                    if vps == 4:
                        x = -2 if is_right_side else 0
                        v1 = self.scene_input.get_vertex(rd, i2 * vps + 2 + x)
                        v2 = self.scene_input.get_vertex(rd, i2 * vps + 3 + x)
                    else:
                        if is_right_side:
                            if n_lanes_other_side == 0:
                                v2 = self.scene_input.get_vertex(rd, i2 * vps + 1)
                            else:
                                v2 = np.divide(np.add(self.scene_input.get_vertex(rd, i2 * vps), self.scene_input.get_vertex(rd, i2 * vps + 1)), 2)
                            v1 = self.scene_input.get_vertex(rd, i2 * vps)
                        else:
                            if n_lanes_other_side == 0:
                                v1 = self.scene_input.get_vertex(rd, i2 * vps)
                            else:
                                v1 = np.divide(np.add(self.scene_input.get_vertex(rd, i2 * vps), self.scene_input.get_vertex(rd, i2 * vps + 1)), 2)
                            v2 = self.scene_input.get_vertex(rd, i2 * vps + 1)
                v3 = [0.0, 0.0, 0.0]
                vT = np.divide(np.subtract(v2, v1), n_lanes)
                vT2 = np.multiply(vT, i + 0.5)
                v3 = np.add(vT2, v1)
                lane.append(v3)
            lanes.append(lane)
        dists = []
        if is_right_side:
            lanes = list(reversed(lanes))  # affects behaviour at intersections
        for l in lanes:
            tot_dist = 0.0
            for iv in range(len(l)):
                v1 = l[iv]
                v2 = l[iv]
                if iv > 0:
                    v2 = l[iv - 1]
                    tot_dist += self.verts_distance(v1, v2)
                file.write_float(tot_dist)
        tot_dist = 0.0
        for i in n_sections_range:
            x = -1 if is_right_side else 0
            y = 1 if is_right_side else -1
            v1 = self.scene_input.get_vertex(rd, (i - x) * vps + x)
            v2 = self.scene_input.get_vertex(rd, (i - x) * vps + x)
            if i != n_sections_range[0]:
                v2 = self.scene_input.get_vertex(rd, (i + y - x) * vps + x)
                tot_dist += self.verts_distance(v1, v2)
            file.write_float(tot_dist)
        for i in range(n_lanes + 5):
            file.write_float(0.0)
        for i in range(6):
            file.write_float(0.0)
        for l in lanes:
            for v in l:
                file.write_vec3(v)
        sw_i = []
        sw_m = []
        sw_o = []
        for i in range(n_sections):
            x = 1 if is_right_side else 0
            y = 0 if not this_side_has_sidewalks else (1 if is_right_side else -1)
            v1 = self.scene_input.get_vertex(rd, (i - x + 1) * vps + x - 1)
            v2 = self.scene_input.get_vertex(rd, (i - x + 1) * vps + x + y - 1)
            v3 = np.divide(np.add(v1, v2), 2)
            sw_i.append(v2)
            sw_o.append(v1)
            sw_m.append(v3)
        for v in sw_m:
            file.write_vec3(v)
        for v in sw_i:
            file.write_vec3(v)
        for v in sw_o:
            file.write_vec3(v)

    def get_bounding_box(self, vertices, add):
        # Returns the bounding box for a set of vertices,
        # with its size increased in each direction by the "add" value
        min_x = float('inf')
        min_y = float('inf')
        min_z = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')
        max_z = float('-inf')
        for v in vertices:
            if v[0] > max_x:
                max_x = v[0]
            if v[0] < min_x:
                min_x = v[0]
            if v[1] > max_y:
                max_y = v[1]
            if v[1] < min_y:
                min_y = v[1]
            if v[2] > max_z:
                max_z = v[2]
            if v[2] < min_z:
                min_z = v[2]
        return [[min_x - add, min_y - add, min_z - add], [max_x + add, max_y + add, max_z + add]]

    def boxes_intersect(self, b1, b2):
        ai = [(b2[0][a] > b1[1][a] or b2[1][a] < b1[0][a]) for a in range(3)]
        return not (ai[0] or ai[1] or ai[2])

    def write(self):
        self.scene_input.init_progress_bar()
        self.file = BinaryFileHelper(self.filepath.replace(".psdl", ".bai"), 'wb')
        file = self.file
        self.roads_finished = []
        self.intersections_finished = []
        print("Getting the BAI objects...")
        obj_count = 0
        for o in self.scene_input.get_object_list():
            o_pr = self.scene_input.get_property_container(o)
            if self.utils.get_object_type(o) == "BAI":
                if bool(int(o_pr["is_road"])):
                    self.roads_finished.append(o)
                else:
                    self.intersections_finished.append(o)
                obj_count += 1.0
        blocks = self.utils.get_real_blocks()
        blocks_o = []
        for b in blocks:
            blocks_o.append([])
        for obj in self.scene_input.get_object_list():
            iblock = self.utils.get_block_number(obj)
            for i in range(len(blocks)):
                if iblock == blocks[i][0]:
                    blocks_o[i].append(obj)
                    break
        print("writing the bai...")
        file.write_raw_string("CAI1")
        file.write_uint16(len(self.intersections_finished))
        file.write_uint16(len(self.roads_finished))
        progress = 0
        progress2 = 0.0
        for i_rd in range(len(self.roads_finished)):
            rd = self.roads_finished[i_rd]
            rd_pr = self.scene_input.get_property_container(rd)
            rd_blocks = rd_pr["blocks"].split(";")
            file.write_uint16(i_rd)
            sec_count = int(rd_pr["vertices_per_section"])
            n_sections = self.scene_input.get_vertices_num(rd) // sec_count
            file.write_uint16(n_sections)
            speed_value = int(rd_pr.get("speed", 0))
            file.write_uint16(speed_value)
            file.write_uint16(len(rd_blocks))
            for rdb in rd_blocks:
                iblock = int(rdb)
                ib = self.utils.get_real_block_number(blocks, iblock)
                file.write_uint16(ib + 1)
            file.write_float(19.0)
            file.write_float(15.0)
            self.write_road_data(rd, True)
            self.write_road_data(rd, False)
            tot_dist = 0.0
            file.write_float(0.0)
            origs = []
            dirs = []
            dirsx = []
            dirsy = []
            dirsz = []
            for i in range(n_sections):
                posA = [0.0, 0.0, 0.0]
                for i2 in range(sec_count):
                    posA_add = self.scene_input.get_vertex(rd, i * sec_count + i2)
                    posA = np.add(posA, posA_add)
                posA = np.divide(posA, sec_count)
                direction = None
                if i < n_sections - 1:
                    posB = [0.0, 0.0, 0.0]
                    for i2 in range(sec_count):
                        posB_add = self.scene_input.get_vertex(rd, (i + 1) * sec_count + i2)
                        posB = np.add(posB, posB_add)
                    posB = np.divide(posB, sec_count)
                    direction = self.utils.get_normal(np.subtract(posB, posA))
                    tot_dist += self.verts_distance(posA, posB)
                    file.write_float(tot_dist)
                else:
                    direction = dirs[-1]
                dirs.append(direction)
                dirx = self.utils.get_normal(np.negative(np.cross(direction, [0, 0, 1])))
                diry = self.utils.get_normal(np.cross(direction, dirx))
                dirsx.append(dirx)
                dirsy.append(diry)
                dirsz.append(np.negative(direction))
                origs.append(posA)
            for a in [dirsx, dirsy, dirsz, dirs]:
                # direction calculated from N to next, but should be to prev instead, here it gets fixed
                e0 = a[0]
                a.insert(0, e0)
                a.pop()
            for v in origs:
                file.write_vec3(v)
            for v in dirsx:
                file.write_vec3(v)
            for v in dirsy:
                file.write_vec3(v)
            for v in dirsz:
                file.write_vec3(v)
            for v in dirs:
                file.write_vec3(v)
            self.write_road_end(rd, 1)
            self.write_road_end(rd, 0)
            progress += 1
            progress2 += 1
            print(str(progress))
            if progress2 % 10 == 0:
                self.scene_input.set_progress_bar(0.99 + (progress2/obj_count) * 0.001)
        progress = 0
        for iIS in range(len(self.intersections_finished)):
            IS = self.intersections_finished[iIS]
            IS_p = self.prop(IS)
            is_roads = IS_p["roads"].split(";")
            iblock = int(IS_p["block"])
            ib = self.utils.get_real_block_number(blocks, iblock)
            file.write_uint16(iIS)
            file.write_uint16(ib + 1)
            file.write_vec3(self.get_intersection_center(IS))
            file.write_uint16(len(is_roads))
            for crs in is_roads:
                for ir in range(len(self.roads_finished)):
                    r = self.roads_finished[ir]
                    if crs == self.prop(r)["id"]:
                        file.write_uint32(ir)
            progress += 1
            progress2 += 1
            print(str(progress))
            if progress2 % 10 == 0:
                self.scene_input.set_progress_bar(0.99 + (progress2/obj_count) * 0.001)
        file.write_uint32(len(blocks) + 1)
        cull_large_list = []
        cull_small_list = []
        large_bubble_size = 200.0
        small_bubble_size = 100.0
        progress = 0.0
        valid_block_types = ["ROADS", "ROADN", "ROADSN", "ROADD", "ROADDN", "BLOCK", "IS", "NUL"]
        road_bounding_boxes_large = []
        road_bounding_boxes_small = []
        for i2 in range(len(self.roads_finished)):
            r = self.roads_finished[i2]
            road_vertices = []
            num_verts_r = self.scene_input.get_vertices_num(r)
            for i_vr in range(num_verts_r):
                vr = self.scene_input.get_vertex(r, i_vr)
                road_vertices.append(vr)
            road_bounding_boxes_large.append(self.get_bounding_box(road_vertices, large_bubble_size / 2))
            road_bounding_boxes_small.append(self.get_bounding_box(road_vertices, small_bubble_size / 2))
        for i in range(len(blocks)):
            cull_large = []
            cull_small = []
            b = blocks_o[i]
            block_vertices = []
            for obj in b:
                objtyp = self.utils.get_object_type(obj)
                if objtyp in valid_block_types:
                    num_verts_obj = self.scene_input.get_vertices_num(obj)
                    for i_vb in range(num_verts_obj):
                        vb = self.scene_input.get_vertex(obj, i_vb)
                        block_vertices.append(vb)
            bounding_box_large = self.get_bounding_box(block_vertices, large_bubble_size / 2)
            bounding_box_small = self.get_bounding_box(block_vertices, small_bubble_size / 2)
            for i2 in range(len(self.roads_finished)):
                road_box = road_bounding_boxes_large[i2]
                intersect = self.boxes_intersect(bounding_box_large, road_box)
                if not intersect:
                    continue
                if self.accurate_culling:
                    # Accurate culling (slower)
                    # Checks the actual distance for each of the two bubbles
                    r = self.roads_finished[i2]
                    found = False
                    found2 = False
                    for obj in b:
                        if found:
                            break
                        num_verts_obj2 = self.scene_input.get_vertices_num(obj)
                        objtyp = self.utils.get_object_type(obj)
                        if objtyp in valid_block_types:
                            for i_vb in range(num_verts_obj2):
                                if found:
                                    break
                                vb = self.scene_input.get_vertex(obj, i_vb)
                                num_verts_r2 = self.scene_input.get_vertices_num(r)
                                for i_vr in range(num_verts_r2):
                                    if found:
                                        break
                                    vr = self.scene_input.get_vertex(r, i_vr)
                                    dist = self.verts_distance(vb, vr)
                                    if dist < small_bubble_size:
                                        found = True
                                        cull_small.append(i2)
                                        if not found2:
                                            found2 = True
                                            cull_large.append(i2)
                                    elif dist < large_bubble_size and not found2:
                                        found2 = True
                                        cull_large.append(i2)
                else:
                    # Approximate culling (faster)
                    # Only does a bounding box check for the small bubble
                    # (the large one has been checked already)
                    cull_large.append(i2)
                    road_box2 = road_bounding_boxes_small[i2]
                    intersect2 = self.boxes_intersect(bounding_box_small, road_box2)
                    if intersect2:
                        cull_small.append(i2)
            cull_large_list.append(cull_large)
            cull_small_list.append(cull_small)
            progress += 1.0
            print(str(progress))
            if progress % 10 == 0:
                self.scene_input.set_progress_bar(0.991 + (progress/len(blocks))*0.009)
        for cullN in [cull_large_list, cull_small_list]:
            for i in range(-1, len(blocks)):
                if i == -1:
                    file.write_uint16(0)
                else:
                    file.write_uint16(len(cullN[i]))
                    for i2 in range(len(cullN[i])):
                        file.write_uint16(cullN[i][i2])
        file.close()
        self.scene_input.end_progress_bar()
        print("BAI Exported!")
