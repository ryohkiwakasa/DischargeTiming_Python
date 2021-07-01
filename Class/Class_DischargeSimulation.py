import pyvista as pv
import numpy as np


class DischargeSimulation:
    def __init__(self, mesh_filename):
        # Target 3D data
        print("--- initialize start --- @DischargeSimulation")
        mesh = pv.read(mesh_filename)
        self.mesh_N = mesh.n_cells
        tmp_array = np.zeros((self.mesh_N, 3, 3))
        for i in range(self.mesh_N):
            tmp = mesh.extract_cells(i)
            tmp_array[i, :, :] = np.array(tmp.points)
        self.polygon_data = tmp_array
        mesh.compute_normals(cell_normals=True, point_normals=False, inplace=True)
        self.mesh_norms = np.array(mesh['Normals'])
        print("--- initialize complete --- @DischargeSimulation")

        # For Simulation parameter
        self.delta_t = 1E-4
        self.Vd = 4000
        self.sim_end_time = 5E-3
        self.height = 10

        # For saving parameter
        self.contanct_data = np.array([])
        self.gap_distance = np.array([])
        self.incidence_angle = np.array([])
        self.head_angle = np.array([])

    def detect_collision_point(self, current_pos, tmp_vec):
        break_flag = False
        pos_list = []
        time_array = np.arange(0, self.sim_end_time + self.delta_t, self.delta_t)

        thresh_agnle = 90
        eggplant_angle = 90
        for t in time_array:
            if t % 1E-3 == 0:
                print("now time: ", t)
            # if t  == self.delta_t*5:
            #     atmp = tarmesh_index
            new_pos = current_pos + tmp_vec
            pos_list.append(current_pos)
            # 使用するmeshの選別 (ある一定距離を超えるmeshとは判定行わない)
            # thresh_mesh_distance = 300  # 100 mm
            tri_center_of_grabvity = (self.polygon_data[:, 0, :] + self.polygon_data[:, 1, :] + self.polygon_data[:, 2, :])/3
            # mesh_distance = np.linalg.norm(self.polygon_data[:, 0, :] - current_pos, axis=1)
            mesh_distance = np.linalg.norm(tri_center_of_grabvity - current_pos, axis=1)
            # target_mesh_index = np.where(mesh_distance <= thresh_mesh_distance)[0]
            target_mesh_index = np.argsort(mesh_distance)[:500]
            # print("current_pos: ", current_pos)
            # print("polygon_datas: ", self.polygon_data[:100, 0, :])
            # print("mesh_distance: ", mesh_distance[:100])
            # print("target_mesh_index: ", target_mesh_index)
            target_polygon_data = self.polygon_data[target_mesh_index, :, :]
            target_mesh_norms = self.mesh_norms[target_mesh_index]

            for mesh_index, tmp_norms in enumerate(target_mesh_norms):
                # print("mesh_index: ", mesh_index)
                pos_P1 = target_polygon_data[mesh_index, 0, :]
                pos_P2 = target_polygon_data[mesh_index, 1, :]
                pos_P3 = target_polygon_data[mesh_index, 2, :]
                pos_gravity = (pos_P1 + pos_P2 + pos_P3)/3
                # v1 = current_pos - pos_P1
                # v2 = new_pos - pos_P1
                v1 = current_pos - pos_gravity
                v2 = new_pos - pos_gravity
                if np.dot(v1, tmp_norms) * np.dot(v2, tmp_norms) <= 0:  # 衝突判定
                    if mesh_index == 0:
                        print("Find collision mesh")
                    # 衝突点座標の計算
                    d1 = abs(np.dot(v1, tmp_norms)) / abs(np.linalg.norm(v1))
                    d2 = abs(np.dot(v2, tmp_norms)) / abs(np.linalg.norm(v2))
                    rate_vec = d1 / (d1 + d2)
                    v3 = (1 - rate_vec) * v1 + rate_vec * v2
                    # contact_point = pos_P1 + v3
                    tmp_contact_point = pos_gravity + v3
                    tmp_eggplant_angle = np.rad2deg(np.arccos(np.dot(tmp_norms*-1, tmp_vec/np.linalg.norm(tmp_vec))))
                    tmp_eggplant_angle = tmp_eggplant_angle if tmp_eggplant_angle < 90 else 180 - tmp_eggplant_angle
                    tarmesh_index = target_mesh_index
                    # 衝突点がポリゴン平面上にあるかを確認
                    # vec11 = pos_P1 - pos_P2
                    # vec12 = pos_P2 - contact_point
                    # vec21 = pos_P2 - pos_P3
                    # vec22 = pos_P3 - contact_point
                    # vec31 = pos_P3 - pos_P1
                    # vec32 = pos_P1 - contact_point
                    vec11 = pos_P2 - pos_P1
                    vec12 = tmp_contact_point - pos_P2
                    vec21 = pos_P3 - pos_P2
                    vec22 = tmp_contact_point - pos_P3
                    vec31 = pos_P1 - pos_P3
                    vec32 = tmp_contact_point - pos_P1
                    CPvec1 = np.cross(vec12, vec11) / tmp_norms
                    CPvec2 = np.cross(vec22, vec21) / tmp_norms
                    CPvec3 = np.cross(vec32, vec31) / tmp_norms
                    sign_array = np.array([np.arccos(np.dot(CPvec1/np.linalg.norm(CPvec1), tmp_norms)) > np.pi/2,
                                           np.arccos(np.dot(CPvec2/np.linalg.norm(CPvec2), tmp_norms)) > np.pi/2,
                                           np.arccos(np.dot(CPvec3/np.linalg.norm(CPvec3), tmp_norms)) > np.pi/2])
                    # print("sign_array", sign_array)
                    # print("mesh_index: ", mesh_index, "check sign", sign_array)
                    # # print("raw of eggplant angle", np.dot(tmp_norms*-1, tmp_vec/np.linalg.norm(tmp_vec)))
                    # print("eggplant angle: ", tmp_eggplant_angle)
                    # sign_array = np.sign([CPvec1[0] / abs(CPvec1[0]), CPvec2[0] / abs(CPvec2[0]),
                    #                       CPvec3[0] / abs(CPvec3[0])])
                    # if (sign_array[:-1] == sign_array[1:]).all():

                    # tmp_distance = np.linalg.norm(tmp_contact_point-current_pos)
                    eggplant_angle = tmp_eggplant_angle if tmp_eggplant_angle < thresh_agnle else eggplant_angle
                    contact_point = tmp_contact_point if tmp_eggplant_angle < thresh_agnle else contact_point
                    # contact_point = tmp_contact_point if tmp_distance < thresh_distance else contact_point
                    # thresh_distance = tmp_distance if tmp_distance < thresh_distance else thresh_distance
                    if sign_array.all() or np.logical_not(sign_array).all():
                        # eggplant_angle = np.arccos(np.dot(tmp_norms*-1, tmp_vec/np.linalg.norm(tmp_vec)))
                        eggplant_angle = tmp_eggplant_angle
                        contact_point = tmp_contact_point
                        # https://qiita.com/dekipon/items/2c15c96ce077a198e8f4
                        print("-------------------------------------------------------")
                        # print("nozzle", nozzle + 1, "\tindex: ", index)
                        # print("~ Progress : ", (nozzle * (dpiTiming.shape[1] + 1) + index) * 100 / (
                        #         (dpiTiming.shape[0] + 1) * (dpiTiming.shape[1] + 1)), "% ~")
                        print("check sign", sign_array)
                        print("eggplant angle: ", eggplant_angle)
                        print("contact_point", contact_point)
                        print("-------------------------------------------------------\n")
                        break_flag = True
                        break
                    else:
                        continue
            if break_flag:
                break
                print("Finish ! Break!")
            else:
                current_pos = new_pos
                # print("--- next time step! --- \n")
                # contact_point = new_pos
                continue

        # out_eggplant_angle = np.rad2deg(eggplant_angle) if np.rad2deg(eggplant_angle) <= 90 else 180-np.rad2deg(eggplant_angle)
        return contact_point, eggplant_angle, pos_list, tarmesh_index

    def get_distance_one_point(self, index, nozzle, Nozzles):
        print("index: ", index)

        target =np.array([Nozzles[nozzle].Xpos[index], Nozzles[nozzle].Ypos[index], Nozzles[nozzle].Zpos[index]])
        tmp_norm_vec = np.array(
            [Nozzles[nozzle].Xnorm[index], Nozzles[nozzle].Ynorm[index], Nozzles[nozzle].Znorm[index]])
        tmp_vec = tmp_norm_vec * self.Vd * self.delta_t
        current_pos = target - tmp_norm_vec * self.height

        # Main Function
        contact_point, eggplant_angle, pos_list, tarmesh_index = self.detect_collision_point(current_pos, tmp_vec)

        distance = np.linalg.norm(contact_point - current_pos)
        print("distance: ", distance, "\n")
        return contact_point, eggplant_angle, distance, pos_list#, tarmesh_index

    def get_distance_one_line(self, label, nozzle, Nozzles, TimingArray):
        print("--- call make_simulation_one_line --- @DischargeSimulation")
        indexes = np.where(TimingArray[nozzle].sub_label == label)[0]
        # print("len of indexes: ", len(indexes))
        contact_data = np.zeros((len(indexes), 3))
        incidence_angle = np.zeros(len(indexes))
        gap_distance = np.zeros(len(indexes))

        for index in indexes:
            print("index: ", index)
            if TimingArray[nozzle].on_off[index] == 0:
                print("pass")
                pass
            else:
                print("exist paint data")
                contact_point, eggplant_angle, distance, pos_list = self.get_distance_one_point(index, nozzle, Nozzles)
                contact_data[index-indexes[0], :] = contact_point
                incidence_angle[index-indexes[0]] = eggplant_angle
                gap_distance[index - indexes[0]] = distance
        print("Complete! get distance one line")
        self.contanct_data = contact_data
        self.gap_distance = gap_distance
        self.incidence_angle = incidence_angle
        aa = np.where(gap_distance  > 15)[0] + indexes[0]
        print("gap dekai: ", aa)
        return contact_data

    def get_distance_one_nozzle(self, nozzle, Nozzles, TimingArray):
        contact_data = np.zeros((TimingArray[0].dataNum, 3))
        incidence_angle = np.zeros(TimingArray[0].dataNum)
        gap_distance = np.zeros(TimingArray[0].dataNum)
        print("TimingArray[0].dataNum: ", TimingArray[0].dataNum)
        print("TimingArray[nozzle].on_off: ", TimingArray[nozzle].on_off.shape)

        for index in range(TimingArray[0].dataNum):
            if TimingArray[nozzle].on_off[index] == 0:
                print("index: ", index)
                print("pass")
                pass
            else:
                print("exist paint data")
                contact_point, eggplant_angle, distance, pos_list = self.get_distance_one_point(index, nozzle, Nozzles)
                contact_data[index, :] = contact_point
                incidence_angle[index] = eggplant_angle
                gap_distance[index] = distance
        print("Complete! -get distance one nozzle-")
        self.contanct_data = contact_data
        self.gap_distance = gap_distance
        self.incidence_angle = incidence_angle
        return contact_data

    def get_distance_all_point(self, Nozzles, TimingArray, delta_t=1E-4):
        contact_data = np.zeros((TimingArray[0].nozzleNum, TimingArray[0].dataNum, 3))
        incidence_angle = np.zeros((TimingArray[0].nozzleNum, TimingArray[0].dataNum))
        gap_distance = np.zeros((TimingArray[0].nozzleNum, TimingArray[0].dataNum))

        for nozzle in range(TimingArray[0].nozzleNum):
            for index in range(TimingArray[0].dataNum):
                if TimingArray[nozzle].on_off[index] == 0:
                    pass
                else:
                    contact_point, eggplant_angle, distance, pos_list = self.get_distance_one_point(index, nozzle, Nozzles)
                    contact_data[nozzle, index, :] = contact_point
                    incidence_angle[nozzle, index] = eggplant_angle
                    gap_distance[nozzle, index] = distance
        print("Complete!")
        self.contanct_data = contact_data
        self.gap_distance = gap_distance
        self.incidence_angle = incidence_angle
        return contact_data

    def make_simulation_one_point(self, index, nozzle, Nozzles, TimingArray):
        print("index: ", index)
        print("delay time:", TimingArray[nozzle].delay_time[index]*1000, "msec\n")
        # time_array = np.arange(0, self.sim_end_time+self.delta_t, self.delta_t)
        # pos_list = []

        target = np.array([Nozzles[nozzle].Xpos[index], Nozzles[nozzle].Ypos[index], Nozzles[nozzle].Zpos[index]])
        tmp_delay_time = TimingArray[nozzle].delay_time[index]
        tmp_Vh = Nozzles[nozzle].Speed[index]
        tmp_main_vec = np.array(
            [Nozzles[nozzle].Xvec[index], Nozzles[nozzle].Yvec[index], Nozzles[nozzle].Zvec[index]])
        tmp_norm_vec = np.array(
            [Nozzles[nozzle].Xnorm[index], Nozzles[nozzle].Ynorm[index], Nozzles[nozzle].Znorm[index]])

        current_pos = target + tmp_Vh * tmp_main_vec * tmp_delay_time  # delay timeも考慮した吐出開始点
        tmp_vec = tmp_main_vec * tmp_Vh * self.delta_t + tmp_norm_vec * self.Vd * self.delta_t
        # break_flag = False

        contact_point, eggplant_angle, pos_list, tarmesh_index = self.detect_collision_point(current_pos, tmp_vec)
        # for t in time_array:
        #     if t % 1E-3 == 0:
        #         print("now time: ", t)
        #     new_pos = current_pos + tmp_vec
        #     pos_list.append(current_pos)
        #     # 使用するmeshの選別 (ある一定距離を超えるmeshとは判定行わない)
        #     # thresh_mesh_distance = 300  # 100 mm
        #     mesh_distance = np.linalg.norm(self.polygon_data[:, 0, :] - current_pos, axis=1)
        #     # target_mesh_index = np.where(mesh_distance <= thresh_mesh_distance)[0]
        #     target_mesh_index = np.argsort(mesh_distance)[:20]
        #     # print("current_pos: ", current_pos)
        #     # print("polygon_datas: ", self.polygon_data[:100, 0, :])
        #     # print("mesh_distance: ", mesh_distance[:100])
        #     # print("target_mesh_index: ", target_mesh_index)
        #     target_polygon_data = self.polygon_data[target_mesh_index, :, :]
        #     target_mesh_norms = self.mesh_norms[target_mesh_index]
        #
        #     for mesh_index, tmp_norms in enumerate(target_mesh_norms):
        #         # print("mesh_index: ", mesh_index)
        #         pos_P1 = target_polygon_data[mesh_index, 0, :]
        #         pos_P2 = target_polygon_data[mesh_index, 1, :]
        #         pos_P3 = target_polygon_data[mesh_index, 2, :]
        #         v1 = current_pos - pos_P1
        #         v2 = new_pos - pos_P1
        #         if np.dot(v1, tmp_norms) * np.dot(v2, tmp_norms) <= 0:  # 衝突判定
        #             print("Find collision mesh")
        #             # 衝突点座標の計算
        #             d1 = abs(np.dot(v1, tmp_norms)) / abs(np.linalg.norm(v1))
        #             d2 = abs(np.dot(v2, tmp_norms)) / abs(np.linalg.norm(v2))
        #             rate_vec = d1 / (d1 + d2)
        #             v3 = (1 - rate_vec) * v1 + rate_vec * v2
        #             contact_point = pos_P1 + v3
        #             eggplant_angle = np.arccos(np.dot(tmp_norms*-1, tmp_vec/np.linalg.norm(tmp_vec)))
        #             tarmesh_index = target_mesh_index
        #             # 衝突点がポリゴン平面上にあるかを確認
        #             vec11 = pos_P1 - pos_P2
        #             vec12 = pos_P2 - contact_point
        #             vec21 = pos_P2 - pos_P3
        #             vec22 = pos_P3 - contact_point
        #             vec31 = pos_P3 - pos_P1
        #             vec32 = pos_P1 - contact_point
        #             CPvec1 = np.cross(vec12, vec11) / tmp_norms
        #             CPvec2 = np.cross(vec22, vec21) / tmp_norms
        #             CPvec3 = np.cross(vec32, vec31) / tmp_norms
        #             sign_array = np.array([np.arccos(np.dot(CPvec1/np.linalg.norm(CPvec1), tmp_norms)) > np.pi/2,
        #                                    np.arccos(np.dot(CPvec2/np.linalg.norm(CPvec2), tmp_norms)) > np.pi/2,
        #                                    np.arccos(np.dot(CPvec3/np.linalg.norm(CPvec3), tmp_norms)) > np.pi/2])
        #             print("sign_array", sign_array)
        #             print("mesh_index: ", mesh_index, "check sign", sign_array)
        #             print("raw of eggplant angle", np.dot(tmp_norms*-1, tmp_vec/np.linalg.norm(tmp_vec)))
        #             print("eggplant angle: ", np.rad2deg(eggplant_angle))
        #             # sign_array = np.sign([CPvec1[0] / abs(CPvec1[0]), CPvec2[0] / abs(CPvec2[0]),
        #             #                       CPvec3[0] / abs(CPvec3[0])])
        #             # if (sign_array[:-1] == sign_array[1:]).all():
        #             if sign_array.all() or np.logical_not(sign_array).all():
        #                 eggplant_angle = np.arccos(np.dot(tmp_norms*-1, tmp_vec/np.linalg.norm(tmp_vec)))
        #                 # https://qiita.com/dekipon/items/2c15c96ce077a198e8f4
        #                 print("-------------------------------------------------------")
        #                 print("nozzle", nozzle + 1, "\tindex: ", index)
        #                 # print("~ Progress : ", (nozzle * (dpiTiming.shape[1] + 1) + index) * 100 / (
        #                 #         (dpiTiming.shape[0] + 1) * (dpiTiming.shape[1] + 1)), "% ~")
        #                 print("check sign", sign_array)
        #                 print("eggplant angle: ", np.rad2deg(eggplant_angle))
        #                 print("contact_point", contact_point)
        #                 # print("elapsed_time:{0}".format((time.time() - start_time) / 60) + "[min]")
        #                 print("-------------------------------------------------------\n")
        #                 # contact_data[nozzle, index, :] = contact_point
        #                 break_flag = True
        #                 break
        #             else:
        #                 continue
        #     if break_flag:
        #         break
        #         print("Finish ! Break!")
        #     else:
        #         current_pos = new_pos
        #         # print("--- next time step! --- \n")
        #         # contact_point = new_pos
        #         continue

        return contact_point, eggplant_angle, pos_list#, tarmesh_index

    def make_simulation_one_line(self, label, nozzle, Nozzles, TimingArray):
        print("--- call make_simulation_one_line --- @DischargeSimulation")
        indexes = np.where(TimingArray[nozzle].sub_label == label)[0]
        # print("len of indexes: ", len(indexes))
        contact_data = np.zeros((len(indexes), 3))

        for index in indexes:
            print("index: ", index)
            if TimingArray[nozzle].on_off[index] == 0:
                print("pass")
                pass
            else:
                print("exist paint data")
                contact_point, eggplant_angle, pos_list = self.make_simulation_one_point(index, nozzle, Nozzles, TimingArray)
                contact_data[index-indexes[0], :] = contact_point
        print("Complete! make simulation one line")
        return contact_data

    def make_simulation_one_nozzle(self, nozzle, Nozzles, TimingArray):
        contact_data = np.zeros((TimingArray[0].dataNum, 3))

        for index in range(TimingArray[0].dataNum):
            if TimingArray[nozzle].on_off[index] == 0:
                print("pass")
                pass
            else:
                print("exist paint data")
                contact_point, eggplant_angle, pos_list = self.make_simulation_one_point(index, nozzle, Nozzles, TimingArray)
                contact_data[index, :] = contact_point
        print("Complete! make simulation one nozzle")
        return contact_data

    def make_simulation_all_point(self, Nozzles, TimingArray, delta_t=1E-4):
        contact_data = np.zeros((TimingArray[0].nozzleNum, TimingArray[0].dataNum, 3))

        for nozzle in range(TimingArray[0].nozzleNum):
            for index in range(TimingArray[0].dataNum):
                if TimingArray[nozzle].on_off[index] == 0:
                    pass
                else:
                    contact_point, eggplant_angle, pos_list = self.make_simulation_one_point(index, nozzle, Nozzles, TimingArray)
                    contact_data[nozzle, index, :] = contact_point
                    # target = np.array([Nozzles[nozzle].Xpos[index], TNozzles[nozzle].Ypos[index], TNozzles[nozzle].Zpos[index]])
                    # tmp_delay_time = TimingArray[nozzle].delay_time[index]
                    # tmp_Vh = Nozzles[nozzle].speed[index]
                    # tmp_main_vec = np.array([Nozzles[nozzle].Xvec[index],  TNozzles[nozzle].Yvec[index],  TNozzles[nozzle].Zvec[index]])
                    # tmp_norm_vec = np.array([Nozzles[nozzle].Xnorm[index], TNozzles[nozzle].Ynorm[index], TNozzles[nozzle].Znorm[index]])
                    #
                    # current_pos = target[0:3] + tmp_Vh * tmp_delay_time  # delay timeも考慮した吐出開始点
                    # tmp_vec = tmp_main_vec * tmp_Vh * delta_t + tmp_norm_vec * Vd * delta_t
                    # break_flag = False
                    # for t in tmp_time:
                    #     # print("now time: ", t)
                    #     new_pos = current_pos + tmp_vec
                    #     # pos_list.append(new_pos)
                    #     # 使用するmeshの選別 (ある一定距離を超えるmeshとは判定行わない)
                    #     thresh_mesh_distance = 100  # 100 mm
                    #     mesh_distance = np.linalg.norm(polygon_data[:, 0, :] - current_pos, axis=1)
                    #     target_mesh_index = np.where(mesh_distance <= thresh_mesh_distance)[0]
                    #     target_polygon_data = polygon_data[target_mesh_index, :, :]
                    #     target_mesh_norms = mesh_norms[target_mesh_index]
                    #
                    #     for mesh_index, tmp_norms in enumerate(target_mesh_norms):
                    #         pos_P1 = target_polygon_data[mesh_index, 0, :]
                    #         pos_P2 = target_polygon_data[mesh_index, 1, :]
                    #         pos_P3 = target_polygon_data[mesh_index, 2, :]
                    #         v1 = current_pos - pos_P1
                    #         v2 = new_pos - pos_P1
                    #         if np.dot(v1, tmp_norms) * np.dot(v2, tmp_norms) <= 0:  # 衝突判定
                    #             # 衝突点座標の計算
                    #             d1 = abs(np.dot(v1, tmp_norms)) / abs(np.linalg.norm(v1))
                    #             d2 = abs(np.dot(v2, tmp_norms)) / abs(np.linalg.norm(v2))
                    #             rate_vec = d1 / (d1 + d2)
                    #             v3 = (1 - rate_vec) * v1 + rate_vec * v2
                    #             contact_point = pos_P1 + v3
                    #             # 衝突点がポリゴン平面上にあるかを確認
                    #             vec11 = pos_P2 - pos_P1
                    #             vec12 = contact_point - pos_P2
                    #             vec21 = pos_P3 - pos_P2
                    #             vec22 = contact_point - pos_P3
                    #             vec31 = pos_P1 - pos_P3
                    #             vec32 = contact_point - pos_P1
                    #             CPvec1 = np.cross(vec11, vec12) / tmp_norms
                    #             CPvec2 = np.cross(vec21, vec22) / tmp_norms
                    #             CPvec3 = np.cross(vec31, vec32) / tmp_norms
                    #             sign_array = np.sign([CPvec1[0] / abs(CPvec1[0]), CPvec2[0] / abs(CPvec2[0]),
                    #                                   CPvec3[0] / abs(CPvec3[0])])
                    #             if (sign_array[:-1] == sign_array[1:]).all():
                    #                 # https://qiita.com/dekipon/items/2c15c96ce077a198e8f4
                    #                 print("-------------------------------------------------------")
                    #                 print("nozzle", nozzle + 1, "\tindex: ", index)
                    #                 print("~ Progress : ", (nozzle * (dpiTiming.shape[1] + 1) + index) * 100 / (
                    #                             (dpiTiming.shape[0] + 1) * (dpiTiming.shape[1] + 1)), "% ~")
                    #                 print("check sign", sign_array)
                    #                 print("contact_point", contact_point, "\n")
                    #                 print("elapsed_time:{0}".format((time.time() - start_time) / 60) + "[min]")
                    #                 print("-------------------------------------------------------\n")
                    #                 contact_data[nozzle, index, :] = contact_point
                    #                 break_flag = True
                    #                 break
                    #             else:
                    #                 continue
                    #     if break_flag:
                    #         break
                    #         print("Finish ! Break!")
                    #     else:
                    #         current_pos = new_pos
                    #         # print("--- next time step! --- \n")
                    #         continue
        print("Complete! make simulation all point")
        return contact_data


