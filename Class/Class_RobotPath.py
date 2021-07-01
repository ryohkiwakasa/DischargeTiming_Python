from Class_BasicData import BasicData
import numpy as np


class RobotPath(BasicData):
    def __init__(self):
        super(RobotPath, self).__init__()
        self.head_angle = np.array([])
        # self.speed = np.array([])
        # self.without_point_s = 3
        # self.without_point_e = 3

    def decide_speed(self, speed_array, delta_t):
        print("--- call decide_speed --- @RobotPath")
        distance_from_speed = speed_array * delta_t
        cumsum_distance_speed = np.insert(np.cumsum(distance_from_speed), 0, 0)
        self.Speed = np.zeros(self.dataNum)
        for idx in range(cumsum_distance_speed.shape[0]-1):
            try:
                index = np.where( (self.cumsum_distance >= cumsum_distance_speed[idx]) & (self.cumsum_distance <= cumsum_distance_speed[idx+1]) )
                self.Speed[index] = speed_array[idx]
            except ValueError:
                pass
        # print("last_index:", index[-1])

    def set_nozzle_array(self, nozzle_array):
        self.nozzle_array = nozzle_array
        self.nozzleNum = nozzle_array.shape[0]

    def get_head_angle(self, rotate_param, big_head=True):
        R = self.rot(rotate_param)
        sita = np.dot(np.array([0, 0, -1]), R) if big_head else np.dot(np.array([0, 1, 0]), R)
        tmp = np.zeros(self.dataNum)
        norm_vecs = np.array([self.Xnorm, self.Ynorm, self.Znorm]).T
        for i in range(self.dataNum):
            rad_angle = np.arccos(np.dot(norm_vecs[i, :], sita))
            tmp[i] = np.rad2deg(rad_angle)
        self.head_angle = tmp

    def decide_speed_new(self, ref_data):
        print("--- call decide_speed --- @RobotPath")
        ref_pos = np.array([ref_data.Xpos,  ref_data.Ypos,  ref_data.Zpos]).T
        start_pos = np.array(
            [self.Xpos[self.without_point_s], self.Ypos[self.without_point_s], self.Zpos[self.without_point_s]])
        end_pos = np.array(
            [self.Xpos[-self.without_point_e], self.Ypos[-self.without_point_e], self.Zpos[-self.without_point_e]])
        search_start = np.linalg.norm(ref_pos - start_pos, axis=1)
        search_end = np.linalg.norm(ref_pos - end_pos, axis=1)
        start_index = np.argmin(search_start)
        end_index = np.argmin(search_end)
        ref_speed = ref_data.Speed[start_index:end_index]
        ref_time = ref_data.time[start_index:end_index]
        ref_pos = ref_pos[start_index:end_index, :]
        ref_vec = ref_pos[1:, :] - ref_pos[:-1, :]
        ref_distance = np.linalg.norm(ref_vec, axis=1)
        cumsum_distance_speed = np.insert(np.cumsum(ref_distance), 0, 0)
        tmp_speed = np.zeros(self.dataNum - (self.without_point_s + self.without_point_e))
        tmp_time = np.zeros(self.dataNum - (self.without_point_s + self.without_point_e))
        tmp_cumsum_distance = self.cumsum_distance[self.without_point_s:-self.without_point_e] - self.cumsum_distance[self.without_point_s]

        print("Raw_cusum", np.max(tmp_cumsum_distance))
        print("Ref_cusum", np.max(cumsum_distance_speed))
        print("Ref_distace_shape", ref_distance.shape)

        for idx in range(cumsum_distance_speed.shape[0]-1):
            try:
                index = np.where((tmp_cumsum_distance >= cumsum_distance_speed[idx]) & (tmp_cumsum_distance <= cumsum_distance_speed[idx+1]))
                tmp_speed[index] = ref_speed[idx]
                tmp_time[index] = ref_time[idx]
            except ValueError:
                pass
        add_speed_s = np.ones(self.without_point_s)
        add_speed_e = np.ones(self.without_point_e)
        add_speed_s = add_speed_s * ref_data.Speed[0]
        add_speed_e = add_speed_e * ref_data.Speed[-1]
        add_time_s = np.zeros(self.without_point_s)
        add_time_e = np.ones(self.without_point_e) * ref_time[-1]
        self.Speed = np.hstack((add_speed_s, tmp_speed, add_speed_e))
        self.time = np.hstack((add_time_s, tmp_time, add_time_e))
        print("speed num", self.Speed.shape[0], "pos num", self.Xpos.shape[0])
        ref_data.without_point_s = start_index
        ref_data.without_point_e = end_index
        return ref_data


    # def resampling60dpi(self, new_instance, thresh, without_point_s=0, without_point_e=0, fix_distance=0.4233):
    #     self.without_point_s = without_point_s
    #     self.without_point_e = without_point_e
    #     # パスの侵入経路と離脱経路を別名保存
    #     tmpTop_X = self.Xpos[:self.without_point_s]
    #     tmpTop_Y = self.Ypos[:self.without_point_s]
    #     tmpTop_Z = self.Zpos[:self.without_point_s]
    #     tmpTop_Xangle = self.Xangle[:self.without_point_s]
    #     tmpTop_Yangle = self.Yangle[:self.without_point_s]
    #     tmpTop_Zangle = self.Zangle[:self.without_point_s]
    #     tmpTop_Xvec = self.Xvec[:self.without_point_s]
    #     tmpTop_Yvec = self.Yvec[:self.without_point_s]
    #     tmpTop_Zvec = self.Zvec[:self.without_point_s]
    #     tmpBottom_X = self.Xpos[-self.without_point_e:]
    #     tmpBottom_Y = self.Ypos[-self.without_point_e:]
    #     tmpBottom_Z = self.Zpos[-self.without_point_e:]
    #     tmpBottom_Xangle = self.Xangle[-self.without_point_e:]
    #     tmpBottom_Yangle = self.Yangle[-self.without_point_e:]
    #     tmpBottom_Zangle = self.Zangle[-self.without_point_e:]
    #     tmpBottom_Xvec = self.Xvec[-self.without_point_e:]
    #     tmpBottom_Yvec = self.Yvec[-self.without_point_e:]
    #     tmpBottom_Zvec = self.Zvec[-self.without_point_e:]
    #     # パスの侵入経路と離脱経路を削除
    #     self.Xpos = self.Xpos[self.without_point_s:-self.without_point_e]
    #     self.Ypos = self.Ypos[self.without_point_s:-self.without_point_e]
    #     self.Zpos = self.Zpos[self.without_point_s:-self.without_point_e]
    #     self.Xangle = self.Xangle[self.without_point_s:-self.without_point_e]
    #     self.Yangle = self.Yangle[self.without_point_s:-self.without_point_e]
    #     self.Zangle = self.Zangle[self.without_point_s:-self.without_point_e]
    #     self.dataNum = self.dataNum - (self.without_point_s + self.without_point_e)
    #     self.Xvec = self.Xvec[self.without_point_s:-self.without_point_e]
    #     self.Yvec = self.Yvec[self.without_point_s:-self.without_point_e]
    #     self.Zvec = self.Zvec[self.without_point_s:-self.without_point_e]
    #
    #     print("--- call resampling60dpi ---")
    #     self.labeling_subdirection(thresh)
    #     for label in range(int(np.max(self.sub_label))):
    #         # print("label: ", label)
    #         index = np.where(self.sub_label == label+1)[0]
    #
    #         target_x = self.Xpos[index]
    #         target_y = self.Ypos[index]
    #         target_z = self.Zpos[index]
    #         target_subdirec = self.sub_label[index]
    #         input_pos = np.array([target_x, target_y, target_z, target_subdirec]).T
    #
    #         target_x_angle = self.Xangle[index]
    #         target_y_angle = self.Yangle[index]
    #         target_z_angle = self.Zangle[index]
    #         input_angle = np.array([target_x_angle, target_y_angle, target_z_angle, target_subdirec]).T
    #
    #         target_cumsum = self.cumsum_distance[self.without_point_s:-self.without_point_e] - self.cumsum_distance[self.without_point_s]
    #         time_data = target_cumsum[index]
    #         time_data = time_data - time_data[0]
    #
    #         tmp_pos = self.CurveFitting3D_plusAlpha(input_pos, time_data, fix_distance)
    #         tmp_angle = self.CurveFitting3D_plusAlpha(input_angle, time_data, fix_distance)
    #
    #         if label == 0:
    #             resample_pos = tmp_pos
    #             resample_angle = tmp_angle
    #         else:
    #             resample_pos = np.vstack((resample_pos, tmp_pos))
    #             resample_angle = np.vstack((resample_angle, tmp_angle))
    #
    #     input_data = np.hstack((resample_pos[:, 0:3], resample_angle[:, 0:3]))
    #     new_instance.set_data(input_data)
    #     new_instance.sub_label = resample_pos[:, 3]
    #
    #     # new_instanceに最初と最後のデータを追記
    #     new_instance.Xpos = np.hstack((tmpTop_X, new_instance.Xpos, tmpBottom_X))
    #     new_instance.Ypos = np.hstack((tmpTop_Y, new_instance.Ypos, tmpBottom_Y))
    #     new_instance.Zpos = np.hstack((tmpTop_Z, new_instance.Zpos, tmpBottom_Z))
    #     new_instance.Xangle = np.hstack((tmpTop_Xangle, new_instance.Xangle, tmpBottom_Xangle))
    #     new_instance.Yangle = np.hstack((tmpTop_Yangle, new_instance.Yangle, tmpBottom_Yangle))
    #     new_instance.Zangle = np.hstack((tmpTop_Zangle, new_instance.Zangle, tmpBottom_Zangle))
    #     new_instance.sub_label = np.hstack((np.zeros(without_point_s), resample_pos[:, 3], np.zeros(without_point_e)))
    #     new_instance.dataNum = new_instance.Xpos.shape[0]
    #     new_instance.calc_mainvector()
    #     new_instance.calc_cumsum_distance()
    #     new_instance.without_point_s = self.without_point_s
    #     new_instance.without_point_e = self.without_point_e
    #     new_instance.calc_mainvector()
    #     new_instance.calc_cumsum_distance()
    #
    #     self.Xpos = np.hstack((tmpTop_X, self.Xpos, tmpBottom_X))
    #     self.Ypos = np.hstack((tmpTop_Y, self.Ypos, tmpBottom_Y))
    #     self.Zpos = np.hstack((tmpTop_Z, self.Zpos, tmpBottom_Z))
    #     self.Xangle = np.hstack((tmpTop_Xangle, self.Xangle, tmpBottom_Xangle))
    #     self.Yangle = np.hstack((tmpTop_Yangle, self.Yangle, tmpBottom_Yangle))
    #     self.Zangle = np.hstack((tmpTop_Zangle, self.Zangle, tmpBottom_Zangle))
    #     self.Xvec = np.hstack((tmpTop_Xvec, self.Xvec, tmpBottom_Xvec))
    #     self.Yvec = np.hstack((tmpTop_Yvec, self.Yvec, tmpBottom_Yvec))
    #     self.Zvec = np.hstack((tmpTop_Zvec, self.Zvec, tmpBottom_Zvec))
    #     self.sub_label = np.hstack((np.zeros(without_point_s), self.sub_label, np.zeros(without_point_e)))
    #     self.dataNum = self.dataNum + (self.without_point_s + self.without_point_e)
    #     return new_instance


