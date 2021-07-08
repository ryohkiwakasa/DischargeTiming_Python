import sys
import numpy as np
import pyvista as pv
import copy
import matplotlib.pyplot as plt

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

RBfilepath = '../../Data/Part_A/InnerPanel/Ver2_0625/DEMO_NAIHAN_0625_2.JBI'
PBfolderpath = '../../Data/Part_A/InnerPanel/Ver2_0625/PaintData/DEMO_NAIHAN_0625_2PAINT_'
speed_filepath = '../../Data/Part_A/InnerPanel/Ver2_0625/DEMO_NAIHAN_0625_2_TOOL0_PL8.csv'
PBfilepath = [PBfolderpath + str(i) + '.JBI' for i in range(3)]

shift_param = [-1020.536, -528.893, -736.453+500]
rotate_param = [-27.142, -36.505, -156.871]

"""
--- ロボットパスの読み込み ---
"""
print("\n ----------- Read Robotpath data --------- \n")
Raw_RB = RB.RobotPath()
Raw_RB.read_datafile([RBfilepath])
Raw_RB.shift_and_rotate_pos(shift_param, rotate_param)
# Raw_RB.convert_angle_to_norm(standard_vector=np.array([0, 1, 0]))
Raw_RB.convert_angle_to_norm(standard_vector=np.array([0, 1, 0]))
# Raw_RB.shift_normal_offset(height=10)
ReData_RB = RB.RobotPath()
ReData_RB = Raw_RB.resampling60dpi(ReData_RB, thresh=50, without_point_s=3, without_point_e=3)
ReData_RB.convert_angle_to_norm(standard_vector=np.array([0, -1, 0]), rotate_param=rotate_param)
# ReData_RB.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]), rotate_param=rotate_param)
# ReData_RB.convert_angle_to_norm(standard_vector=np.array([1, 0, 0]), rotate_param=rotate_param)
ReData_RB.get_head_angle(rotate_param=rotate_param, big_head=False)

# 速度プラファイルとの対応後
# ReData_RB.adjust_main_vec()
# ReData_RB.caks_subvector()


"""
--- 塗装点の読み込み ---
"""
print("\n ----------- Read paintData data --------- \n")
Raw_PD = PD.PaintData()
Raw_PD.read_datafile(PBfilepath)
Raw_PD.shift_and_rotate_pos(shift_param, rotate_param)
ReData_PB = PD.PaintData()
ReData_PB = Raw_PD.resampling60dpi(ReData_PB, thresh=50, without_point_s=3, without_point_e=3)


"""
--- 速度プロファイルの読み込み ---
"""
print("\n ----------- Read Robotpath data --------- \n")
shift_param[2] = shift_param[2] - 500
Raw_SP = SP.SpeedProfile()
Raw_SP.read_from_csv(speed_filepath)
# Raw_SP.delte_data_point(550, 180)
Raw_SP.shift_and_rotate_pos(shift_param, rotate_param)
# Raw_SP.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]))
# Raw_SP.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]))
Re_SP = SP.SpeedProfile()
Re_SP = Raw_SP.resampling_all_data(Re_SP, re_delta_t=1E-3)
Re_SP.convert_angle_to_norm(standard_vector=np.array([0, 0, 1]))
# Re_SP.convert_angle_to_norm(standard_vector=np.array([0, 1, 0]))


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
nozzles.make_NNV_Head_small()
nozzles.set_center_by_mean()
ReData_RB.set_nozzle_array(nozzles.nozzle_array)
Nozzles = []
for i in range(nozzles.nozzle_num):
    print("Nozzle: ", i+1)
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
for i in range(nozzles.nozzle_num):
    print("\nCreate timing Data for nozzle", i+1)
    dpiTiming = DT.DischargeTiming()
    dpiTiming.set_data(path_data=Nozzles[i], paint_data=ReData_PB, nozzle_id=i)
    dpiTiming.pred_simple_collision_pos()
    dpiTiming.decide_simple_OnOff(direction=False)
    # S_pos, E_pos, S_idx, E_idx = dpiTiming.decide_start_pos_and_end_pos(direction=False)
    TimingDatas.append(dpiTiming)
    # S_idx_list.append(S_idx)
    # E_idx_list.append(E_idx)


# """
# --- Simulationの実行 ---
# """
# print("\n ----------- Execute Simulation --------- \n")
# sim_target_nozzle = 1  # 0~7
# # sim_target_label = 6
# sim_target_label = 3
# target_3d = '../../Data/Demo_ver1/ASSY_hokan.stl'
# # target_3d = '../../Data/Demo_ver1/ASSY_hokan_Voxel_2m.stl'
# sim = DS.DischargeSimulation(target_3d)
# # sim.Vd = -4000
# sim.delta_t = 5E-4
# # sim.delta_t = 7E-4
# # liquid_locus, eggplant_angle, pos_list, taget_index = sim.make_simulation_one_point(index=2295, nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)
# # liquid_locus, eggplant_angle, distance, pos_list, target_index = sim.get_distance_one_point(index=5000, nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)
# # liquid_locus = sim.get_distance_one_line(label=sim_target_label, nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)
# liquid_locus = sim.get_distance_one_nozzle(nozzle=sim_target_nozzle, Nozzles=Nozzles, TimingArray=TimingDatas)


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
# out_filename = "InnerPlate_nozzle_" + str(sim_target_nozzle+1) + ".xlsx"
# print(out_filename)
# Function_Utility.output_env_param_excel(array=output_array, filename=out_filename)


"""
--- pyvista による3Dビューワー ---
"""
print("\n ----------- 3D viewer by pyvista --------- \n")
filename = '../../3DModel/ASSY_hokan_smooth_rough090.stl'
# mesh = pv.read(filename)
mesh = pv.read(filename)

# shift_param_2 = [320, 100, 0]
# rotate_param_2 = [0, 0, 0]
# Raw_SP.shift_and_rotate_pos(shift_param_2, rotate_param_2)
# # Raw_SP.shift_normal_offset(height=10)
# ReData_RB.shift_normal_offset(height=10)

# Robot Path
RobotData_Raw = np.array([Raw_RB.Xpos, Raw_RB.Ypos, Raw_RB.Zpos]).T
RobotData_Re = np.array([ReData_RB.Xpos, ReData_RB.Ypos, ReData_RB.Zpos]).T
n_nozzle = sim_target_nozzle if "sim_target_nozzle" in locals() else 0
NozzlePos = np.array([Nozzles[n_nozzle].Xpos, Nozzles[n_nozzle].Ypos, Nozzles[n_nozzle].Zpos]).T
# paint Data
FigData_Raw = np.array([Raw_PD.Xpos, Raw_PD.Ypos, Raw_PD.Zpos]).T
FigData_Re = np.array([ReData_PB.Xpos, ReData_PB.Ypos, ReData_PB.Zpos]).T
# Speed Profile
SPData_Raw = np.array([Raw_SP.Xpos, Raw_SP.Ypos, Raw_SP.Zpos]).T
# Arrow data
# Arrow_Re = np.array([ReData_RB.Xvec, ReData_RB.Yvec, ReData_RB.Zvec]).T
# Arrow_Re = np.array([ReData_RB.Xsubvec, ReData_RB.Ysubvec, ReData_RB.Zsubvec]).T
Arrow_Re = np.array([ReData_RB.Xnorm, ReData_RB.Ynorm, ReData_RB.Znorm]).T
Arrow_Re = Arrow_Re * 1.5

plotter = pv.Plotter()
color_list = ["red", "blue", "green", "pink", "purple"]
plotter.add_mesh(mesh, color='white')

plotter.add_mesh(pv.PolyData(FigData_Re), color='green', render_points_as_spheres=True)
# for i in range(nozzles.nozzle_num):
#     nozzlePath = np.array([Nozzles[i].Xpos, Nozzles[i].Ypos, Nozzles[i].Zpos]).T
#     plotter.add_mesh(pv.PolyData(nozzlePath), color="gray", point_size=3, render_points_as_spheres=True)
#     plotter.add_mesh(pv.PolyData(nozzlePath[3, :]), color=color_list[i%5], point_size=4,
#                      render_points_as_spheres=True)
#     plotter.add_mesh(pv.PolyData(nozzlePath[-4, :]), color=color_list[i % 5], point_size=4,
#                      render_points_as_spheres=True)

# NozzlePos = NozzlePos - Arrow_Re * 10 / 1.5
plotter.add_mesh(pv.PolyData(NozzlePos), color="gray", point_size=3, render_points_as_spheres=True)
# plotter.add_mesh(pv.PolyData(nozzlePath[S_idx[sim_target_nozzle]]), color="red", point_size=8, render_points_as_spheres=True)
# plotter.add_mesh(pv.PolyData(nozzlePath[E_idx[sim_target_nozzle]]), color="blue", point_size=8, render_points_as_spheres=True)

# plotter.add_mesh(pv.PolyData(liquid_locus), color='blue', point_size=5, render_points_as_spheres=True)
# plotter.add_mesh(pv.PolyData(pos_list), color='black', point_size=4, render_points_as_spheres=True)

plotter.add_arrows(cent=RobotData_Re, direction=Arrow_Re)


plotter.show()




