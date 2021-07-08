import sys
import numpy as np
import pyvista as pv
import copy
import matplotlib.pyplot as plt
import pandas as pd
import csv

sys.path.append('../../Class')
# import Class_BasicData as BD
import Class_My3D as My3D
import Class_PaintData as PD
import Class_RobotPath as RB
import Class_SpeedProfile as SP
import Class_DischargeTiming as DT
import Class_HeadArray as HA
import Class_DischargeSimulation as DS
import Function_Utility

RBfilepath = '../../Data/Part_A/Roof/Ver2_0625/DEMO_ROOF_UPPER_0625_3.JBI'
PBfolderpath = '../../Data/Part_A/Roof/Ver2_0625/PaintData/DEMO_ROOF_UPPER_0625_3PAINT_'
speed_filepath = '../../Data/Part_A/Roof/Ver2_0625/demo_roof_upper_0625_3.csv'
PBfilepath = [PBfolderpath + str(i) + '.JBI' for i in range(70)]

shift_param = [-1020.536, -528.893, -236.453+500]
rotate_param = [-27.142, -36.505, -156.871]

"""
--- ロボットパスの読み込み ---
"""
print("\n ----------- Read Robotpath data --------- \n")
Raw_RB = RB.RobotPath()
Raw_RB.read_datafile([RBfilepath])
Raw_RB.shift_and_rotate_pos(shift_param, rotate_param)
Raw_RB.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]))
# Raw_RB.shift_normal_offset(height=10)
ReData_RB = RB.RobotPath()
ReData_RB = Raw_RB.resampling60dpi(new_instance=ReData_RB, thresh=50, without_point_s=2, without_point_e=2)
ReData_RB.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]), rotate_param=rotate_param)
ReData_RB.get_head_angle(rotate_param=rotate_param)


"""
--- 塗装点の読み込み ---
"""
print("\n ----------- Read paintData data --------- \n")
Raw_PD = PD.PaintData()
Raw_PD.read_datafile(PBfilepath)
Raw_PD.shift_and_rotate_pos(shift_param, rotate_param)
ReData_PB = PD.PaintData()
ReData_PB = Raw_PD.resampling60dpi(new_instance=ReData_PB, thresh=50, without_point_s=2, without_point_e=2)


print("51 ReData_RB.sub_label", ReData_RB.sub_label)
"""
--- 速度プロファイルの読み込み ---
"""
print("\n ----------- Read Robotpath data --------- \n")
shift_param[2] = shift_param[2] - 500
Raw_SP = SP.SpeedProfile()
Raw_SP.read_from_csv(speed_filepath)
# Raw_SP.delte_data_point(550, 180)
Raw_SP.shift_and_rotate_pos(shift_param, rotate_param)
Raw_SP.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]))
Re_SP = SP.SpeedProfile()
Re_SP = Raw_SP.resampling_all_data(Re_SP, re_delta_t=1E-3)
Re_SP.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]))


"""
--- 速度プロファイルとの対応 ---
"""
Re_SP = ReData_RB.decide_speed_new(Re_SP)
ReData_RB.adjust_main_vec()
ReData_RB.caks_subvector()


"""
--- ノズルデータの作成 ---
"""
print("\n ----------- Create Nozzle Position Data  --------- \n")
nozzles = HA.HeadArray()
nozzles.make_NNV_Head()
nozzles.set_center_by_mean()
ReData_RB.set_nozzle_array(nozzles.nozzle_array)
Nozzles = []
for i in range(nozzles.nozzle_num):
    tmp = copy.deepcopy(ReData_RB)
    tmp = ReData_RB.support_nozzle_pos(tmp, i)
    Nozzles.append(tmp)


"""
# タイミングデータの作成
# """
print("\n ----------- Create Timing Data --------- \n")
TimingDatas = []
# for i in range(1):
S_idx_list = []
E_idx_list = []
wait_time = []
for i in range(nozzles.nozzle_num):
    print("\nCreate timing Data for nozzle", i+1)
    dpiTiming = DT.DischargeTiming()
    Nozzles[i].shift_normal_offset(height=10)  # ロボットパスが10 mm浮かした位置を基準にしているので面と同じにする
    dpiTiming.set_data(path_data=Nozzles[i], paint_data=ReData_PB, nozzle_id=i)
    dpiTiming.pred_simple_collision_pos()
    dpiTiming.decide_simple_OnOff(direction=False)
    # start_poses, end_poses, start_index, end_index = dpiTiming.decide_start_pos_and_end_pos(direction=False)
    TimingDatas.append(dpiTiming)
    s_index = dpiTiming.start_index[np.nonzero(dpiTiming.start_index)]
    e_index = dpiTiming.end_index[np.nonzero(dpiTiming.end_index)]
    print("start_index: ", s_index, "\nend_insex: ", e_index)
    tmp_wait_time = []
    for idx in range(e_index.shape[0]-1):
        print("s_index: ", s_index[idx+1], "\te_index: ", e_index[idx])
        time = ReData_RB.time[s_index[idx+1]] - ReData_RB.time[e_index[idx]]
        print("wait time: ", time)
        tmp_wait_time.append(time)
    wait_time.append(tmp_wait_time)
    # print("nozzle: ", i+1, "\nwait time: ", tmp_wait_time)


i = 1
# np.get_printoptions()
# np.set_printoptions(precision=3)
# np.get_printoptions()
for time_print in wait_time:
    print("nozzle: ", i, "\nwait time: ", pd.DataFrame(time_print).round(2))
    i += 1

# i = 1
# with open("wait_head_time.csv", 'w',  newline="") as f:
#     writer = csv.writer(f)
#     for time_print in wait_time:
#         writer.writerow(time_print)
#         print("nozzle: ", i, "\nwait time: ", pd.DataFrame(time_print).round(2))
#         i += 1
# f.close()


# print("S_idx_list", S_idx_list[0])
# outcsvfile = "Demo_ver1_test_0621.csv"
# Function_Utility.output_timing_data(TimingDatas, outcsvfile)


# """
# --- Simulationの実行 ---
# """
# print("\n ----------- Execute Simulation --------- \n")
# sim_target_nozzle = 0  # 0~47
# # sim_target_label = 6
# sim_target_label = 12
# target_3d = '../../Data/Demo_ver1/ASSY_hokan.stl'
# # target_3d = '../../Data/Demo_ver1/ASSY_hokan_Voxel_2m.stl'
# sim = DS.DischargeSimulation(target_3d)
# sim.delta_t = 5E-4
# # sim.delta_t = 7E-4
# # liquid_locus, eggplant_angle, pos_list, taget_index = sim.make_simulation_one_point(index=2295, nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)
# # liquid_locus, eggplant_angle, distance, pos_list, target_index = sim.get_distance_one_point(index=28495, nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)
# # liquid_locus = sim.get_distance_one_line(label=sim_target_label, nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)
# liquid_locus = sim.get_distance_one_nozzle(nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)
#
# """
# データの保存
# """
# gap_distance = sim.gap_distance[sim.gap_distance != 0]
# incidence_angle = sim.incidence_angle[sim.incidence_angle != 0]
# m_gap_distance = Function_Utility.eliminate_error_value(gap_distance)
# m_incidence_angle = Function_Utility.eliminate_error_value(incidence_angle)
# target_index = np.where(sim.gap_distance != 0)[0]
# head_angle = ReData_RB.head_angle[target_index]
# output_array = np.array([m_gap_distance, m_incidence_angle, head_angle]).T
# out_filename = "nozzle_" + str(sim_target_nozzle+1) + ".xlsx"
# print(out_filename)
# Function_Utility.output_env_param_excel(array=output_array, filename=out_filename)



"""
--- pyvista による3Dビューワー ---
"""
print("\n ----------- 3D viewer by pyvista --------- \n")
filename = '../../Data/Demo_ver1/ASSY_hokan.stl'
# filename = '../../Data/Demo_ver1/ASSY_hokan_Voxel_2m.stl'
# mesh = pv.read(filename)
mesh = pv.read(filename)

shift_param_2 = [320, 100, 0]
rotate_param_2 = [0, 0, 0]
# Re_SP.shift_and_rotate_pos(shift_param_2, rotate_param_2)
# Raw_SP.shift_normal_offset(height=10)
ReData_RB.shift_normal_offset(height=-10)
# TimingDatas[sim_target_nozzle].path_data.shift_normal_offset(height=-10)

# Robot Path
RobotData_Raw = np.array([Raw_RB.Xpos, Raw_RB.Ypos, Raw_RB.Zpos]).T
RobotData_Re = np.array([ReData_RB.Xpos, ReData_RB.Ypos, ReData_RB.Zpos]).T
n_nozzle = 23
NozzlePos = np.array([Nozzles[n_nozzle].Xpos, Nozzles[n_nozzle].Ypos, Nozzles[n_nozzle].Zpos]).T
# paint Data
# FigData_Raw = np.array([Raw_PD.Xpos, Raw_PD.Ypos, Raw_PD.Zpos]).T
FigData_Re = np.array([ReData_PB.Xpos, ReData_PB.Ypos, ReData_PB.Zpos]).T
# Speed Profile
# SPData_Raw = np.array([Raw_SP.Xpos, Raw_SP.Ypos, Raw_SP.Zpos]).T
SPData_Re = np.array([Re_SP.Xpos, Re_SP.Ypos, Re_SP.Zpos]).T
# Arrow data
# Arrow_Re = np.array([ReData_RB.Xvec, ReData_RB.Yvec, ReData_RB.Zvec]).T
# Arrow_Re = np.array([ReData_RB.Xsubvec, ReData_RB.Ysubvec, ReData_RB.Zsubvec]).T
Arrow_Re = np.array([ReData_RB.Xnorm, ReData_RB.Ynorm, ReData_RB.Znorm]).T
Arrow_Re = Arrow_Re * 5

print("Reaed\t",
      "speed num", RobotData_Re[ReData_RB.without_point_s:-ReData_RB.without_point_e, :].shape[0],
      "pos num", ReData_RB.Speed[ReData_RB.without_point_s:-ReData_RB.without_point_e].shape[0])

plotter = pv.Plotter()
color_list = ["red", "blue", "green", "pink", "purple"]
plotter.add_mesh(mesh, color='white', show_edges=False)
# plotter.add_mesh(pv.PolyData(SPData_Raw[s_index:e_index, :]), color="blue", point_size=4, render_points_as_spheres=True, show_scalar_bar=True)
# plotter.add_mesh(pv.PolyData(RobotData_Re), color="gray", point_size=2, render_points_as_spheres=True, show_scalar_bar=True)
plotter.add_mesh(pv.PolyData(NozzlePos), color="gray", point_size=2, render_points_as_spheres=True, show_scalar_bar=True)
# plotter.add_arrows(cent=NozzlePos, direction=Arrow_Re)
# plotter.add_mesh(pv.PolyData(NozzlePos[start_index]), color="red", point_size=6, render_points_as_spheres=True, show_scalar_bar=True)
# plotter.add_mesh(pv.PolyData(NozzlePos[end_index]), color="blue", point_size=6, render_points_as_spheres=True, show_scalar_bar=True)
plotter.add_mesh(pv.PolyData(FigData_Re), color="green", point_size=2, render_points_as_spheres=True, show_scalar_bar=True)
# plotter.add_mesh(pv.PolyData(start_poses), color="red", point_size=6, render_points_as_spheres=True, show_scalar_bar=True)
# plotter.add_mesh(pv.PolyData(end_poses), color="blue", point_size=6, render_points_as_spheres=True, show_scalar_bar=True)

# plotter.add_mesh(pv.PolyData(S_pos), color="red", point_size=8, render_points_as_spheres=True, show_scalar_bar=True)
# plotter.add_mesh(pv.PolyData(E_pos), color="blue", point_size=8, render_points_as_spheres=True, show_scalar_bar=True)
# plotter.add_mesh(pv.PolyData(liquid_locus), color='blue', point_size=5, render_points_as_spheres=True)
# plotter.add_mesh(pv.PolyData(pos_list), color='black', point_size=4, render_points_as_spheres=True)
plotter.add_arrows(cent=RobotData_Re, direction=Arrow_Re)

plotter.show()




