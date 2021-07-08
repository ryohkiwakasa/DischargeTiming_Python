import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import re


class BasicData:
    def __init__(self):
        self.dataNum = 0
        self.nozzleNum = 0
        self.nozzle_id = 0
        self.nozzle_array = np.array([])
        self.Xpos = np.array([])
        self.Ypos = np.array([])
        self.Zpos = np.array([])
        self.Xangle = np.array([])
        self.Yangle = np.array([])
        self.Zangle = np.array([])
        self.Xvec = np.array([])
        self.Yvec = np.array([])
        self.Zvec = np.array([])
        self.Xnorm = np.array([])
        self.Ynorm = np.array([])
        self.Znorm = np.array([])
        self.Xsubvec = np.array([])
        self.Ysubvec = np.array([])
        self.Zsubvec = np.array([])
        self.sub_label = np.array([])
        self.time = np.array([])
        self.Speed = np.array([])
        self.distance = np.array([])
        self.cumsum_distance = np.array([])
        self.thresh_subvec = 0
        self.flag_index = np.array([])
        self.without_point_s = 3
        self.without_point_e = 3

    def Sub_Read_StringRowData(self, filepath):
        f = open(filepath, 'r')
        row_No = 0
        str_data = []
        while True:
            line = f.readline()
            if line:
                row_No += 1
                str_data.append(line)
            else:
                break
        f.close()
        return str_data

    def Sub_ReadRobotCode(self, filepath, file_num):
        str_data = self.Sub_Read_StringRowData(filepath)
        i = -1
        data = []
        while True:
            i += 1
            try:
                line = str_data[i]
                try:
                    tmp = re.split('[=]', line)
                    if len(tmp[1]) > 10:
                        data.append(list(map(float, tmp[1].split(','))))
                except:
                    pass
            except IndexError:  # 最後まで読み終わっている場合
                print('file No.', file_num, ' Complete !')
                break
        return np.array(data)

    def calc_mainvector(self):
        tmpX = self.Xpos[1:] - self.Xpos[:-1]
        tmpY = self.Ypos[1:] - self.Ypos[:-1]
        tmpZ = self.Zpos[1:] - self.Zpos[:-1]
        self.Xvec = np.insert(tmpX, -1, tmpX[-1])
        self.Yvec = np.insert(tmpY, -1, tmpY[-1])
        self.Zvec = np.insert(tmpZ, -1, tmpZ[-1])

    def adjust_main_vec(self):
        print("self.sub_label: ", self.sub_label)
        for label in range(int(np.max(self.sub_label))):
            print("label: ", label+1)
            index = np.where(self.sub_label == label+1)[0]
            self.Xvec[index[-1]] = self.Xvec[index[-2]]
            self.Yvec[index[-1]] = self.Yvec[index[-2]]
            self.Zvec[index[-1]] = self.Zvec[index[-2]]


    def calc_cumsum_distance(self):
        self.distance = np.sqrt(self.Xvec[:-1]**2 + self.Yvec[:-1]**2 + self.Zvec[:-1]**2)
        self.distance = np.insert(self.distance, 0, 0)
        # self.cumsum_distance = np.insert(np.cumsum(self.distance), 0, 0)
        self.cumsum_distance = np.cumsum(self.distance)

    def set_data(self, data):
        self.dataNum = data.shape[0]
        self.Xpos = data[:, 0]
        self.Ypos = data[:, 1]
        self.Zpos = data[:, 2]
        self.Xangle = data[:, 3]
        self.Yangle = data[:, 4]
        self.Zangle = data[:, 5]
        self.calc_mainvector()
        self.calc_cumsum_distance()
        # self.distance = np.array([np.sqrt( (self.Xpos[i+1] - self.Xpos[i])**2 + (self.Ypos[i+1] - self.Ypos[i])**2 + (self.Zpos[i+1] - self.Zpos[i]) ** 2) for i in range(self.dataNum-1)])
        # self.cumsum_distance = np.insert(np.cumsum(self.distance), 0, 0)

    def delte_data_point(self, delete_point_start, delete_point_end):
        self.dataNum = self.dataNum - (delete_point_start+delete_point_end)
        self.time = self.time[delete_point_start:-delete_point_end]
        self.Xpos = self.Xpos[delete_point_start:-delete_point_end]
        self.Ypos = self.Ypos[delete_point_start:-delete_point_end]
        self.Zpos = self.Zpos[delete_point_start:-delete_point_end]
        self.Xangle = self.Xangle[delete_point_start:-delete_point_end]
        self.Yangle = self.Yangle[delete_point_start:-delete_point_end]
        self.Zangle = self.Zangle[delete_point_start:-delete_point_end]
        self.Speed = self.Speed[delete_point_start:-delete_point_end]
        self.calc_mainvector()

    def shift_normal_offset(self, height):
        self.Xpos = self.Xpos + self.Xnorm * height
        self.Ypos = self.Ypos + self.Ynorm * height
        self.Zpos = self.Zpos + self.Znorm * height
        self.calc_mainvector()

    def rot(self, p):
        # https://org-technology.com/posts/rotational-transformation-matrix.html
        # 回転行列を計算する
        px = p[0] * (np.pi / 180)
        py = p[1] * (np.pi / 180)
        pz = p[2] * (np.pi / 180)

        # 物体座標系の 1->2->3 軸で回転させる
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(px), np.sin(px)],
                       [0, -np.sin(px), np.cos(px)]])
        Ry = np.array([[np.cos(py), 0, -np.sin(py)],
                       [0, 1, 0],
                       [np.sin(py), 0, np.cos(py)]])
        Rz = np.array([[np.cos(pz), np.sin(pz), 0],
                       [-np.sin(pz), np.cos(pz), 0],
                       [0, 0, 1]])
        R = Rz.dot(Ry).dot(Rx)
        return R

    def shift_and_rotate_pos(self, shift_param, rotate_param):
        # Shift
        self.Xpos = self.Xpos + shift_param[0]
        self.Ypos = self.Ypos + shift_param[1]
        self.Zpos = self.Zpos + shift_param[2]

        # Rotate
        R = self.rot(rotate_param)
        Raw_pos = np.array([self.Xpos, self.Ypos, self.Zpos]).T
        Rotate_pos = np.dot(Raw_pos, R)
        self.Xpos = Rotate_pos[:, 0]
        self.Ypos = Rotate_pos[:, 1]
        self.Zpos = Rotate_pos[:, 2]

        # # Angle Change
        # self.Xangle = self.Xangle + rotate_param[0]
        # self.Yangle = self.Yangle + rotate_param[1]
        # self.Zangle = self.Zangle + rotate_param[2]

        # modify vec data
        self.calc_mainvector()


    def read_datafile(self, filelist):
        sFlag = True
        file_num = 1
        for file in filelist:
            if sFlag:
                data = self.Sub_ReadRobotCode(file, file_num)
                sFlag = False
            else:
                tmp = self.Sub_ReadRobotCode(file, file_num)
                data = np.vstack((data, tmp))
            file_num += 1
        self.set_data(data)

    def convert_angle_to_norm(self, standard_vector, rotate_param=np.array([0.0, 0.0, 0.0]), not_reverse=True):
        phi_x = (2 * np.pi / 360) * self.Xangle
        # phi_x = (2 * np.pi / 360) * self.Xangle * -1
        phi_y = (2 * np.pi / 360) * self.Yangle
        # phi_y = (2 * np.pi / 360) * self.Yangle * -1
        phi_z = (2 * np.pi / 360) * self.Zangle
        # phi_z = (2 * np.pi / 360) * self.Zangle * -1
        one_array = np.ones(phi_x.shape)
        zero_array = np.zeros(phi_x.shape)
        queue_x = np.array([[one_array, zero_array, zero_array],
                            [zero_array, np.cos(phi_x), -np.sin(phi_x)],
                            [zero_array, np.sin(phi_x), np.cos(phi_x)]])
        queue_y = np.array([[np.cos(phi_y), zero_array, np.sin(phi_y)],
                            [zero_array, one_array, zero_array],
                            [-np.sin(phi_y), zero_array, np.cos(phi_y)]])
        queue_z = np.array([[np.cos(phi_z), -np.sin(phi_z), zero_array],
                            [np.sin(phi_z), np.cos(phi_z), zero_array],
                            [zero_array, zero_array, one_array]])
        tmp = np.dot(standard_vector, queue_x)
        for idx in range(queue_y.shape[2]):
            tmp[:, idx] = np.dot(tmp[:, idx], queue_y[:, :, idx])
        for idx in range(queue_z.shape[2]):
            tmp[:, idx] = np.dot(tmp[:, idx], queue_z[:, :, idx])

        R = self.rot(rotate_param)
        tmp = np.dot(tmp.T, R).T

        self.Xnorm = tmp[0, :] if not_reverse else -1*tmp[0, :]
        self.Ynorm = tmp[1, :] if not_reverse else -1*tmp[1, :]
        self.Znorm = tmp[2, :] if not_reverse else -1*tmp[2, :]

    def polar2cartesian(self, position):
        # https://moromi-senpy.hatenablog.com/entry/2019/08/06/174039
        # rθΦ座標からxyz座標への変換
        num = position.shape[0]
        newPosition = np.empty([num, 3], dtype=np.float64)
        newPosition[:, 0] = position[:, 0] * np.sin(position[:, 1]) * np.cos(position[:, 2])
        newPosition[:, 1] = position[:, 0] * np.sin(position[:, 1]) * np.sin(position[:, 2])
        newPosition[:, 2] = position[:, 0] * np.cos(position[:, 1])
        return newPosition

    def cartesian2polar(self, position):
        # https://moromi-senpy.hatenablog.com/entry/2019/08/06/174039
        # xyz座標からrθΦ座標への変換
        num = position.shape[0]
        newPosition = np.empty([num, 3], dtype=np.float64)
        newPosition[:, 0] = np.linalg.norm(position, axis=1)
        newPosition[:, 1] = np.arccos(position[:, 2] / newPosition[:, 0])
        newPosition[:, 2] = np.arctan2(position[:, 1], position[:, 0])
        nan_index = np.isnan(newPosition[:, 1])
        newPosition[nan_index, 1] = 0
        newPosition[:, 1] = newPosition[:, 1] * (360/np.pi)
        newPosition[:, 2] = newPosition[:, 2] * (360/np.pi)
        return newPosition

    def sub_specifyVecChangePoint(self, thresh):
        print("--- call sub_specifyVecChangePoint ---")
        self.thresh_subvec = thresh
        tmp_vec = self.cartesian2polar(np.array([self.Xvec, self.Yvec, self.Zvec]).T)

        diff_vecData = abs(tmp_vec[1:, 1:] - tmp_vec[:-1, 1:])  # 角度データのみ使用

        plt.figure()
        plt.plot(diff_vecData[:, 0], "b.")
        plt.plot(diff_vecData[:, 1], "g.")
        plt.title('Check diff vec data theta')
        plt.legend(["Δθ", "Δφ"])
        plt.grid()
        plt.show()

        # tmp_flag = np.where((diff_vecData[:, 0] > thresh) | (diff_vecData[:, 1] > thresh), True, False)
        tmp_flag = np.where((diff_vecData[:, 0] + diff_vecData[:, 1] > thresh), True, False)
        flag_index = np.array(
            [True if tmp_flag[i] == True and tmp_flag[i + 1] == True else False for i in range(tmp_flag.shape[0] - 1)])
        self.flag_index = np.insert(flag_index, 0, tmp_flag[0])

    def sub_specifyVecChangePoint_new(self, thresh):
        print("--- call sub_specifyVecChangePoint ---")
        self.thresh_subvec = thresh
        tmp_vec = np.array([self.Xvec, self.Yvec, self.Zvec]).T
        for i in range(tmp_vec.shape[0]):
            tmp_vec[i, :] = tmp_vec[i, :] / np.linalg.norm(tmp_vec[i])
        # print("tmp_vec", tmp_vec[-10:])
        tmp_vec[-1] = tmp_vec[-2]

        diff_vecData = np.zeros(tmp_vec.shape[0])
        for i in range(tmp_vec.shape[0]-1):
            tmp_inner = np.dot(tmp_vec[i+1, :], tmp_vec[i, :])
            diff_vecData[i] = np.rad2deg(np.arccos(tmp_inner))

        # plt.figure()
        # plt.plot(diff_vecData, "b.")
        # plt.title('Check diff vec data theta')
        # plt.legend(["Δθ", "Δφ"])
        # plt.grid()
        # plt.show()

        # tmp_flag = np.where((diff_vecData[:, 0] > thresh) | (diff_vecData[:, 1] > thresh), True, False)
        tmp_flag = np.where((diff_vecData > self.thresh_subvec), True, False)
        flag_index = np.array(
            [True if np.logical_xor(tmp_flag[i], tmp_flag[i+1]) and tmp_flag[i+1] else False for i in range(tmp_flag.shape[0]-1)])
        print("length of flag_index: ", len(flag_index))
        # self.flag_index = np.insert(flag_index, 0, tmp_flag[0])
        self.flag_index = np.roll(flag_index, 2)

    def labeling_subdirection(self, thresh):
        # まずflagを求める
        print("--- call labeling_subdirection ---")
        self.sub_specifyVecChangePoint_new(thresh)
        indexes = np.where(self.flag_index == True)[0]
        tmpArray = np.zeros(self.flag_index.shape)
        labelNum = 1
        for item in indexes:  # indexesの配列の形なんか特徴的
            # item -= 1
            if labelNum == 1:
                tmpArray[:item] = labelNum
                labelNum += 1
            else:
                if abs(item - past_item) <= 2:
                    tmpArray[past_item:item] = labelNum + 1
                else:
                    tmpArray[past_item:item] = labelNum
                    labelNum += 1
            past_item = item
        tmpArray[item:] = labelNum  # 最後の処理が必要
        print("total number of label: ", labelNum)
        self.sub_label = np.insert(tmpArray, 0, 1)

    def CurveFitting3D_plusAlpha(self, input_data, ref_data, fix_space):
        # 線形補完した後で，固定幅で配列を作成
        fit_x = interp1d(ref_data, input_data[:, 0], kind='linear')
        fit_y = interp1d(ref_data, input_data[:, 1], kind='linear')
        fit_z = interp1d(ref_data, input_data[:, 2], kind='linear')
        fit_l = interp1d(ref_data, input_data[:, 3], kind='linear')
        space_data = np.arange(ref_data[0], ref_data[-1], fix_space)
        output = np.array([fit_x(space_data), fit_y(space_data), fit_z(space_data), fit_l(space_data)])
        return output.T
    #
    def resampling60dpi_Basic(self, thresh, fix_distance=0.4233):
        print("--- call resampling60dpi ---")
        self.labeling_subdirection(thresh)
        for label in range(int(np.max(self.sub_label))):
            # print("label: ", label)
            index = np.where(self.sub_label == label + 1)[0]

            target_x = self.Xpos[index]
            target_y = self.Ypos[index]
            target_z = self.Zpos[index]
            target_subdirec = self.sub_label[index]
            input_pos = np.array([target_x, target_y, target_z, target_subdirec]).T

            target_x_angle = self.Xangle[index]
            target_y_angle = self.Yangle[index]
            target_z_angle = self.Zangle[index]
            input_angle = np.array([target_x_angle, target_y_angle, target_z_angle, target_subdirec]).T

            target_cumsum = self.cumsum_distance[self.without_point_s:-self.without_point_e] - self.cumsum_distance[
                self.without_point_s]
            time_data = target_cumsum[index]
            time_data = time_data - time_data[0]

            tmp_pos = self.CurveFitting3D_plusAlpha(input_pos, time_data, fix_distance)
            tmp_angle = self.CurveFitting3D_plusAlpha(input_angle, time_data, fix_distance)

            if label == 0:
                resample_pos = tmp_pos
                resample_angle = tmp_angle
            else:
                resample_pos = np.vstack((resample_pos, tmp_pos))
                resample_angle = np.vstack((resample_angle, tmp_angle))

        # input_data = np.hstack((resample_pos[:, 0:3], resample_angle[:, 0:3]))
        # new_instance.set_data(input_data)
        # new_instance.sub_label = resample_pos[:, 3]
        return resample_pos, resample_angle

    def resampling60dpi(self, new_instance, thresh, without_point_s=0, without_point_e=0, fix_distance=0.4233):
        self.without_point_s = without_point_s
        self.without_point_e = without_point_e
        # パスの侵入経路と離脱経路を別名保存
        tmpTop_X = self.Xpos[:self.without_point_s]
        tmpTop_Y = self.Ypos[:self.without_point_s]
        tmpTop_Z = self.Zpos[:self.without_point_s]
        tmpTop_Xangle = self.Xangle[:self.without_point_s]
        tmpTop_Yangle = self.Yangle[:self.without_point_s]
        tmpTop_Zangle = self.Zangle[:self.without_point_s]
        tmpTop_Xvec = self.Xvec[:self.without_point_s]
        tmpTop_Yvec = self.Yvec[:self.without_point_s]
        tmpTop_Zvec = self.Zvec[:self.without_point_s]
        tmpBottom_X = self.Xpos[-self.without_point_e:]
        tmpBottom_Y = self.Ypos[-self.without_point_e:]
        tmpBottom_Z = self.Zpos[-self.without_point_e:]
        tmpBottom_Xangle = self.Xangle[-self.without_point_e:]
        tmpBottom_Yangle = self.Yangle[-self.without_point_e:]
        tmpBottom_Zangle = self.Zangle[-self.without_point_e:]
        tmpBottom_Xvec = self.Xvec[-self.without_point_e:]
        tmpBottom_Yvec = self.Yvec[-self.without_point_e:]
        tmpBottom_Zvec = self.Zvec[-self.without_point_e:]
        # パスの侵入経路と離脱経路を削除
        self.Xpos = self.Xpos[self.without_point_s:-self.without_point_e]
        self.Ypos = self.Ypos[self.without_point_s:-self.without_point_e]
        self.Zpos = self.Zpos[self.without_point_s:-self.without_point_e]
        self.Xangle = self.Xangle[self.without_point_s:-self.without_point_e]
        self.Yangle = self.Yangle[self.without_point_s:-self.without_point_e]
        self.Zangle = self.Zangle[self.without_point_s:-self.without_point_e]
        self.dataNum = self.dataNum - (self.without_point_s + self.without_point_e)
        self.Xvec = self.Xvec[self.without_point_s:-self.without_point_e]
        self.Yvec = self.Yvec[self.without_point_s:-self.without_point_e]
        self.Zvec = self.Zvec[self.without_point_s:-self.without_point_e]

        # Main Function (Look only hear!)
        resample_pos, resample_angle = self.resampling60dpi_Basic(thresh=thresh, fix_distance=fix_distance)

        # 入力データの作成
        input_data = np.hstack((resample_pos[:, 0:3], resample_angle[:, 0:3]))
        new_instance.set_data(input_data)
        # new_instance.sub_label = resample_pos[:, 3]

        # new_instanceに最初と最後のデータを追記
        new_instance.Xpos = np.hstack((tmpTop_X, new_instance.Xpos, tmpBottom_X))
        new_instance.Ypos = np.hstack((tmpTop_Y, new_instance.Ypos, tmpBottom_Y))
        new_instance.Zpos = np.hstack((tmpTop_Z, new_instance.Zpos, tmpBottom_Z))
        new_instance.Xangle = np.hstack((tmpTop_Xangle, new_instance.Xangle, tmpBottom_Xangle))
        new_instance.Yangle = np.hstack((tmpTop_Yangle, new_instance.Yangle, tmpBottom_Yangle))
        new_instance.Zangle = np.hstack((tmpTop_Zangle, new_instance.Zangle, tmpBottom_Zangle))
        new_instance.sub_label = np.hstack((np.zeros(without_point_s), resample_pos[:, 3], np.zeros(without_point_e)))
        new_instance.dataNum = new_instance.Xpos.shape[0]
        new_instance.calc_mainvector()
        new_instance.calc_cumsum_distance()
        new_instance.without_point_s = self.without_point_s
        new_instance.without_point_e = self.without_point_e
        new_instance.calc_mainvector()
        new_instance.calc_cumsum_distance()

        self.Xpos = np.hstack((tmpTop_X, self.Xpos, tmpBottom_X))
        self.Ypos = np.hstack((tmpTop_Y, self.Ypos, tmpBottom_Y))
        self.Zpos = np.hstack((tmpTop_Z, self.Zpos, tmpBottom_Z))
        self.Xangle = np.hstack((tmpTop_Xangle, self.Xangle, tmpBottom_Xangle))
        self.Yangle = np.hstack((tmpTop_Yangle, self.Yangle, tmpBottom_Yangle))
        self.Zangle = np.hstack((tmpTop_Zangle, self.Zangle, tmpBottom_Zangle))
        self.Xvec = np.hstack((tmpTop_Xvec, self.Xvec, tmpBottom_Xvec))
        self.Yvec = np.hstack((tmpTop_Yvec, self.Yvec, tmpBottom_Yvec))
        self.Zvec = np.hstack((tmpTop_Zvec, self.Zvec, tmpBottom_Zvec))
        self.sub_label = np.hstack((np.zeros(without_point_s), self.sub_label, np.zeros(without_point_e)))
        self.dataNum = self.dataNum + (self.without_point_s + self.without_point_e)
        return new_instance

    def calcOrthogonal_Vector(self, a, b):
        ax, ay, az = a[:, 0], a[:, 1], a[:, 2]
        bx, by, bz = b[:, 0], b[:, 1], b[:, 2]
        c1 = [(-ay * bz + az * by) * np.sqrt(1 / (
                    ax ** 2 * by ** 2 + ax ** 2 * bz ** 2 - 2.0 * ax * ay * bx * by - 2.0 * ax * az * bx * bz + ay ** 2 * bx ** 2 + ay ** 2 * bz ** 2 - 2.0 * ay * az * by * bz + az ** 2 * bx ** 2 + az ** 2 * by ** 2)),
              (ax * bz - az * bx) * np.sqrt(1 / (
                          ax ** 2 * by ** 2 + ax ** 2 * bz ** 2 - 2.0 * ax * ay * bx * by - 2.0 * ax * az * bx * bz + ay ** 2 * bx ** 2 + ay ** 2 * bz ** 2 - 2.0 * ay * az * by * bz + az ** 2 * bx ** 2 + az ** 2 * by ** 2)),
              (-ax * by + ay * bx) * np.sqrt(1 / (
                          ax ** 2 * by ** 2 + ax ** 2 * bz ** 2 - 2.0 * ax * ay * bx * by - 2.0 * ax * az * bx * bz + ay ** 2 * bx ** 2 + ay ** 2 * bz ** 2 - 2.0 * ay * az * by * bz + az ** 2 * bx ** 2 + az ** 2 * by ** 2))]
        c2 = [(ay * bz - az * by) * np.sqrt(1 / (
                    ax ** 2 * by ** 2 + ax ** 2 * bz ** 2 - 2.0 * ax * ay * bx * by - 2.0 * ax * az * bx * bz + ay ** 2 * bx ** 2 + ay ** 2 * bz ** 2 - 2.0 * ay * az * by * bz + az ** 2 * bx ** 2 + az ** 2 * by ** 2)),
              (-ax * bz + az * bx) * np.sqrt(1 / (
                          ax ** 2 * by ** 2 + ax ** 2 * bz ** 2 - 2.0 * ax * ay * bx * by - 2.0 * ax * az * bx * bz + ay ** 2 * bx ** 2 + ay ** 2 * bz ** 2 - 2.0 * ay * az * by * bz + az ** 2 * bx ** 2 + az ** 2 * by ** 2)),
              (ax * by - ay * bx) * np.sqrt(1 / (
                          ax ** 2 * by ** 2 + ax ** 2 * bz ** 2 - 2.0 * ax * ay * bx * by - 2.0 * ax * az * bx * bz + ay ** 2 * bx ** 2 + ay ** 2 * bz ** 2 - 2.0 * ay * az * by * bz + az ** 2 * bx ** 2 + az ** 2 * by ** 2))]
        c1 = np.array(c1).T
        c2 = np.array(c2).T
        return [c1, c2]

    def caks_subvector(self, dir=True):
        main_vec = np.array([self.Xvec, self.Yvec, self.Zvec]).T
        norm_vec = np.array([self.Xnorm, self.Ynorm, self.Znorm]).T
        tmp = self.calcOrthogonal_Vector(main_vec, norm_vec)
        sub_vec = tmp[0] if dir == True else tmp[1]
        self.Xsubvec = sub_vec[:, 0]
        self.Ysubvec = sub_vec[:, 1]
        self.Zsubvec = sub_vec[:, 2]

    def support_nozzle_pos(self, new_instance, nozzle_id):
        print("nozzle_id: ", nozzle_id)
        main_vec = np.array([self.Xvec, self.Yvec, self.Zvec])
        sub_vec = np.array([self.Xsubvec, self.Ysubvec, self.Zsubvec]).T
        main_vec = np.transpose(main_vec / np.linalg.norm(main_vec, axis=0))
        polarity = self.sub_label % 2
        polarity = polarity*(-2)+1
        nozzle_vec = sub_vec*self.nozzle_array[nozzle_id, 0] + main_vec*self.nozzle_array[nozzle_id, 1]
        for i in range(nozzle_vec.shape[0]):
            nozzle_vec[i, :] = nozzle_vec[i, :] * polarity[i]

        new_instance.Xpos = self.Xpos + nozzle_vec[:, 0]
        new_instance.Ypos = self.Ypos + nozzle_vec[:, 1]
        new_instance.Zpos = self.Zpos + nozzle_vec[:, 2]
        new_instance.cumsum_distanmce = self.cumsum_distance
        new_instance.nozzle_id = nozzle_id
        return new_instance



