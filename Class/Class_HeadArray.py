import numpy as np
import csv
import copy


class HeadArray:
    def __init__(self):
        self.nozzle_num = 0
        self.nozzle_array = np.array([])
        self.center_x = 0
        self.center_y = 0
        self.center_z = 0

    def make_NNV_Head_small(self):
        one_nozzle_num = 8
        one_nozzle = np.zeros((one_nozzle_num, 3))

        theta = np.deg2rad(-2.965)
        x_diffs_nozzle_raw = 0.0
        y_diffs_nozzle_raw = 7.694
        x_diffs_nozzle = x_diffs_nozzle_raw * np.cos(theta) + y_diffs_nozzle_raw * np.sin(theta)
        y_diffs_nozzle = y_diffs_nozzle_raw * np.cos(theta) + x_diffs_nozzle_raw * np.sin(theta)

        for i in range(one_nozzle_num):
            one_nozzle[i, 0] = x_diffs_nozzle * i
            one_nozzle[i, 1] = y_diffs_nozzle * i

        self.nozzle_array = one_nozzle
        self.nozzle_num = one_nozzle.shape[0]
        self.center_x = np.mean(one_nozzle[:, 0])
        self.center_y = np.mean(one_nozzle[:, 1])


    def make_NNV_Head(self):
        one_nozzle_num = 8
        one_nozzle = np.zeros((one_nozzle_num, 3))

        theta = np.deg2rad(-9.59)
        x_diffs_nozzle_raw = 0.0
        y_diffs_nozzle_raw = 7.694
        x_diffs_nozzle = x_diffs_nozzle_raw * np.cos(theta) + y_diffs_nozzle_raw * np.sin(theta)
        y_diffs_nozzle = y_diffs_nozzle_raw * np.cos(theta) + x_diffs_nozzle_raw * np.sin(theta)

        for i in range(one_nozzle_num):
            one_nozzle[i, 0] = x_diffs_nozzle * i
            one_nozzle[i, 1] = y_diffs_nozzle * i

        head_num = 3
        x_diffs_head_raw = np.array([0, 12.75, 25.5])
        y_diffs_head_raw = np.array([0, 72.9, 145.8])
        x_diffs_head = x_diffs_head_raw * np.cos(theta) + y_diffs_head_raw * np.sin(theta)
        y_diffs_head = y_diffs_head_raw * np.cos(theta) + x_diffs_head_raw * np.sin(theta)
        for j in range(head_num):
            tmp_nozzle = copy.copy(one_nozzle)
            tmp_nozzle[:, 0] = tmp_nozzle[:, 0] - x_diffs_head[j]
            tmp_nozzle[:, 1] = tmp_nozzle[:, 1] + y_diffs_head[j]
            if j == 0:
                nozzle_array = copy.copy(tmp_nozzle)
            else:
                nozzle_array = np.vstack((nozzle_array, tmp_nozzle))
        print("nozzle_shape", nozzle_array.shape)

        # 回転中心を変更
        tmp_center_x = np.mean(nozzle_array[:, 0])
        tmp_center_y = np.mean(nozzle_array[:, 1])
        nozzle_array[:, 0] = nozzle_array[:, 0] - tmp_center_x
        nozzle_array[:, 1] = nozzle_array[:, 1] - tmp_center_y

        # R = np.array([[np.cos(theta), -1 * np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        # nozzle_array[:, :2] = np.dot(nozzle_array[:, :2], R)

        print("nozzle_shape", nozzle_array.shape)
        self.nozzle_array = nozzle_array
        self.nozzle_num = nozzle_array.shape[0]
        self.center_x = np.mean(nozzle_array[:, 0])
        self.center_y = np.mean(nozzle_array[:, 1])

    def Archive_make_NNV_Head(self):
        one_nozzle_num = 8
        x_diffs_nozzle = 0.0
        y_diffs_nozzle = 7.694
        one_nozzle = np.zeros((one_nozzle_num, 3))

        for i in range(one_nozzle_num):
            one_nozzle[i, 0] = x_diffs_nozzle * i
            one_nozzle[i, 1] = y_diffs_nozzle * i

        head_num = 3
        x_diffs_head = [0, 12.75, 25.5]
        y_diffs_head = [0, 72.9, 145.8]
        for j in range(head_num):
            tmp_nozzle = copy.copy(one_nozzle)
            tmp_nozzle[:, 0] = tmp_nozzle[:, 0] - x_diffs_head[j]
            tmp_nozzle[:, 1] = tmp_nozzle[:, 1] + y_diffs_head[j]
            if j == 0:
                nozzle_array = copy.copy(tmp_nozzle)
            else:
                nozzle_array = np.vstack((nozzle_array, tmp_nozzle))
        print("nozzle_shape", nozzle_array.shape)

        theta = np.deg2rad(9.59)
        R = np.array([[np.cos(theta), -1 * np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        nozzle_array[:, :2] = np.dot(nozzle_array[:, :2], R)

        print("nozzle_shape", nozzle_array.shape)
        self.nozzle_array = nozzle_array
        self.nozzle_num = nozzle_array.shape[0]
        self.center_x = np.mean(nozzle_array[:, 0])
        self.center_y = np.mean(nozzle_array[:, 1])

    def make_nozzle_array(self, Nozzle_XY = [-5.08, 5.778], y_Head = 22.6,
                          x_diffs=np.array([20.64, 22.33, 18.95, 21.49, 23.18, 19.79]) - 20.64):
        x_nozzle, y_nozzle = Nozzle_XY[0], Nozzle_XY[1]
        nozzles = np.zeros((8, 3))
        for i in range(8):
            nozzles[i, 0] = -x_nozzle * i
            nozzles[i, 1] = (i % 2) * y_nozzle

        # ヘッドのノズル配列の作成
        HeadNozzles = np.copy(nozzles)
        # x_diffs = np.array([20.64, 22.33, 18.95, 21.49, 23.18, 19.79]) - 20.64
        for j in range(len(x_diffs) - 1):
            tmp = np.copy(nozzles)
            tmp[:, 0] = nozzles[:, 0] + x_diffs[j + 1]
            tmp[:, 1] = nozzles[:, 1] + y_Head * (j + 1)
            HeadNozzles = np.vstack((HeadNozzles, tmp))
        self.nozzle_array = HeadNozzles
        self.nozzle_num = 48
        self.center_x = np.mean(HeadNozzles[:, 0])
        self.center_y = np.mean(HeadNozzles[:, 1])

    def set_center_by_mean(self):
        self.nozzle_array[:, 0] = self.nozzle_array[:, 0] - self.center_x
        self.nozzle_array[:, 1] = self.nozzle_array[:, 1] - self.center_y
        self.nozzle_array[:, 2] = self.nozzle_array[:, 2] - self.center_z

    def set_center_by_specificpoint(self, center_pos):
        self.nozzle_array[:, 0] = self.nozzle_array[:, 0] - center_pos[0]
        self.nozzle_array[:, 1] = self.nozzle_array[:, 1] - center_pos[1]
        self.nozzle_array[:, 2] = self.nozzle_array[:, 2] - center_pos[2]
        self.center_x = center_pos[0]
        self.center_y = center_pos[1]
        self.center_z = center_pos[2]

    def output_csv(self, filepath):
        with open(filepath, 'w', newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(self.nozzle_array)
        csv_file.close()
        print("Complete output nozzle array data as csv file.")
