import numpy as np
import copy
from scipy.interpolate import interp1d
# from scipy.signal import decimate
from Class_PaintData import PaintData
from Class_RobotPath import RobotPath


class DischargeTiming:
    def __init__(self):
        self.print_data = PaintData()
        self.path_data = RobotPath()
        self.dataNum = 0
        self.nozzle_id = 0
        self.Vd = 4000
        self.gap_space = 10
        self.sub_label = np.array([])
        self.pred_collision = np.array([])
        self.collision_pos = np.array([])
        self.time_array = np.array([])
        self.pred_collision_time = np.array([])
        self.on_off = np.array([])
        self.delay_time = np.array([])
        self.open_time = np.array([])

        self.start_index = np.array([])
        self.end_index = np.array([])

    def set_data(self, path_data, paint_data=PaintData(), nozzle_id=0):
        self.print_data = paint_data
        self.path_data = path_data
        self.dataNum = path_data.Xpos.shape[0]
        self.nozzle_id = nozzle_id
        self.sub_label = path_data.sub_label

    def make_time_array(self):
        tmp_time = self.path_data.distance[:-1] / self.path_data.speed[:-1]
        self.time_array = np.insert(np.cumsum(tmp_time), 0, 0)

    def pred_simple_collision_point(self):
        self.pred_collision = self.path_data.cumsum_distance + self.path_data.speed * (self.gap_space/self.Vd)

    def pred_simple_collision_pos(self):
        delta_t = self.gap_space / self.Vd
        tmp_pos = np.array([self.path_data.Xpos, self.path_data.Ypos, self.path_data.Zpos]).T
        main_vec = np.array([self.path_data.Xvec, self.path_data.Yvec, self.path_data.Zvec]).T
        main_vec = main_vec / np.linalg.norm(main_vec)
        for i in range(main_vec.shape[0]):
            main_vec[i, :] = main_vec[i, :] * self.path_data.Speed[i] * delta_t
        self.collision_pos = tmp_pos + main_vec
        print("shape of collision_pos", self.collision_pos.shape, "tmppos shape", tmp_pos.shape)

    def decide_pred_collision_time(self, start_distance, end_distance, distance_step=0.4233):
        self.make_time_array()
        self.pred_simple_collision_point()

        discharge_point_array = np.arange(start_distance, end_distance, distance_step)
        for label in range(int(np.max(self.path_data.sub_label))):
            porality = ((label+1)%2 * (-2)) + 1
            # porality = (label % 2 * (-2)) + 1
            index = np.where(self.path_data.sub_label == label+1)
            tmp_pred = self.pred_collision[index]
            tmp_pred = tmp_pred - tmp_pred[0] + self.path_data.nozzle_array[self.path_data.nozzle_id, 1] * porality
            tmp_time = self.time_array[index]
            fit_collision = interp1d(tmp_pred, tmp_time, kind='linear')
            tmp_collision_time = fit_collision(discharge_point_array)
            # tmp_collision_time = decimate(tmp_collision_time, dimension)
            if label == 0:
                collision_time = tmp_collision_time
            else:
                collision_time = np.hstack((collision_time, tmp_collision_time))
        self.pred_collision_time = collision_time

    def Archive_decide_start_pos_and_end_pos_1(self):
        start_poses = []
        end_poses = []
        for label in range(int(np.max(self.print_data.sub_label))):
            index = np.where(self.print_data.sub_label == label+1)[0]
            tmp_pos = np.array([self.print_data.Xpos[index], self.print_data.Ypos[index], self.print_data.Zpos[index]]).T
            if (label+1) % 2 == 1:
                start_poses.append(tmp_pos[0, :])
                end_poses.append(tmp_pos[-1, :])
            else:
                start_poses.append(tmp_pos[-1, :])
                end_poses.append(tmp_pos[0, :])
        start_poses = np.array(start_poses)
        end_poses = np.array(end_poses)

        start_index = np.array([])
        end_index = np.array([])
        for label in range(int(np.max(self.path_data.sub_label))):
            print("-----------------------------------")
            print("start label: ", label+1)
            thresh_angle = 20

            flag_start_end = np.array([False, False])
            index = np.where(self.path_data.sub_label == label + 1)[0]
            tmp_pos = np.array([self.path_data.Xpos[index], self.path_data.Ypos[index], self.path_data.Zpos[index]]).T
            tmp_vec = np.array([self.path_data.Xvec[index], self.path_data.Yvec[index], self.path_data.Zvec[index]]).T
            tmp_start_poses = copy.copy(start_poses)
            tmp_end_poses = copy.copy(end_poses)
            for i in range(tmp_pos.shape[0]-1):
                now_pos = tmp_pos[i, :]
                # next_pos = tmp_pos[i+1, :]
                main_vec = tmp_vec[i, :] / np.linalg.norm(tmp_vec[i, :])
                if flag_start_end.all():
                    print("-----------------------------------")
                    print("Roop end \t label: ", label+1,  "\n")
                    break
                else:
                    if not flag_start_end[0]:
                        now_vec = tmp_start_poses - now_pos if (label+1)%2==1 else tmp_end_poses - now_pos
                        now_vec_norm = now_vec.T/np.linalg.norm(now_vec, axis=1)
                        tmp_internal = np.dot(now_vec_norm.T, main_vec)
                        flag = True if np.min(tmp_internal) < 0.0 else False
                        if flag:
                            flag_start_end[0] = True
                            start_index = np.append(start_index, index[i-1])
                            print("Find start pos", "\t label: ", label+1, "index: ", index[i-1])
                            print("flag_start_end: ", flag_start_end)
                    else:
                        now_vec = tmp_end_poses - now_pos if (label+1)%2==1 else tmp_start_poses - now_pos
                        now_vec_norm = now_vec.T / np.linalg.norm(now_vec, axis=1)
                        tmp_internal = np.dot(now_vec_norm.T, main_vec)
                        flag = True if np.min(tmp_internal) < 0.0 else False
                        if flag:
                            flag_start_end[1] = True
                            end_index = np.append(end_index, index[i-1])
                            print("Find end pos", "\t label: ", label + 1, "index: ", index[i-1])
                            print("flag_start_end: ", flag_start_end)
                # delete some start_pos & end_pos
                tmp_angle = np.rad2deg(np.arccos(tmp_internal))
                # print("angle s", abs(tmp_angle))
                # print("angle shape", tmp_angle.shape)
                target_idx = np.where(abs(tmp_angle) < thresh_angle)[0]
                tmp_start_poses = tmp_start_poses[target_idx]
                tmp_end_poses = tmp_end_poses[target_idx]
                thresh_angle += 1
        print("\nstart_index: \n", start_index.astype(int), "\nend_index: \n", end_index.astype(int), "\n")

        return start_poses, end_poses, start_index.astype(int), end_index.astype(int)

    def Archive_decide_start_pos_and_end_pos_2(self):
        start_poses = []
        end_poses = []
        for label in range(int(np.max(self.print_data.sub_label))):
            index = np.where(self.print_data.sub_label == label+1)[0]
            tmp_pos = np.array([self.print_data.Xpos[index], self.print_data.Ypos[index], self.print_data.Zpos[index]]).T
            if (label+1) % 2 == 1:
                start_poses.append(tmp_pos[0, :])
                end_poses.append(tmp_pos[-1, :])
            else:
                start_poses.append(tmp_pos[-1, :])
                end_poses.append(tmp_pos[0, :])
        start_poses = np.array(start_poses)
        end_poses = np.array(end_poses)

        thresh_distance = 10
        support_index = np.zeros(int(np.max(self.path_data.sub_label)))
        start_index = np.zeros(int(np.max(self.path_data.sub_label)))
        end_index = np.zeros(int(np.max(self.path_data.sub_label)))
        for label in range(int(np.max(self.path_data.sub_label))):
            index = np.where(self.path_data.sub_label == label + 1)[0]
            tmp_pos = np.array([self.path_data.Xpos[index], self.path_data.Ypos[index], self.path_data.Zpos[index]]).T
            distance_s = 1E8

            for i in range(start_poses.shape[0]):
                # print("tmp_pos.shape", tmp_pos.shape)
                # print("start_poses", start_poses[i], "end_poses", end_poses[i])
                target = tmp_pos - start_poses[i] if (label+1) % 2 == 1 else tmp_pos - end_poses[i]
                Dis_S = np.linalg.norm(target, axis=1)
                min_Dis_s = np.min(Dis_S)
                # print("min_Dis_s", min_Dis_s)
                if min_Dis_s < distance_s:
                    distance_s = min_Dis_s
                    support_index[label] = i + 1

            if np.logical_and(np.logical_not(support_index[label] == 0), distance_s <= thresh_distance):
                target_idx = int(support_index[label] - 1)
                print("target_idx: ", target_idx)
                print("Find! at label: ", label + 1)
                print("min_Distance: ", distance_s, " mm")
                Dis_S = np.linalg.norm(tmp_pos - start_poses[target_idx], axis=1) if (label + 1) % 2 == 1 else np.linalg.norm(tmp_pos - end_poses[target_idx], axis=1)
                Dis_E = np.linalg.norm(tmp_pos - end_poses[target_idx], axis=1) if (label + 1) % 2 == 1 else np.linalg.norm(tmp_pos - start_poses[target_idx], axis=1)
                start_index[label] = index[0] + np.argmin(Dis_S)
                end_index[label] = index[0] + np.argmin(Dis_E)
                print("start_index: ", index[0] + np.argmin(Dis_S), "end_index: ", index[0] + np.argmin(Dis_E), "\n")
            else:
                # start_index[label] = np.append(start_index, index[0])
                # end_index[label] = np.append(end_index, index[0])
                print("index none ... ", "\n")

        # print("\nstart_index: \n", start_index.astype(int), "\nend_index: \n", end_index.astype(int), "\n")
        return start_poses, end_poses, start_index.astype(int), end_index.astype(int)

    def decide_start_pos_and_end_pos(self, direction=True):
        start_poses = []
        end_poses = []
        for label in range(int(np.max(self.print_data.sub_label))):
            index = np.where(self.print_data.sub_label == label+1)[0]
            tmp_pos = np.array([self.print_data.Xpos[index], self.print_data.Ypos[index], self.print_data.Zpos[index]]).T
            if (label+1) % 2 == 1:
                start_poses.append(tmp_pos[0, :])
                end_poses.append(tmp_pos[-1, :])
            else:
                start_poses.append(tmp_pos[-1, :])
                end_poses.append(tmp_pos[0, :])
        start_poses = np.array(start_poses)
        end_poses = np.array(end_poses)

        thresh_angle = 5
        support_index = np.zeros(int(np.max(self.path_data.sub_label)))
        start_index = np.zeros(int(np.max(self.path_data.sub_label)))
        end_index = np.zeros(int(np.max(self.path_data.sub_label)))
        for label in range(int(np.max(self.path_data.sub_label))):
            index = np.where(self.path_data.sub_label == label + 1)[0]
            # tmp_pos = np.array([self.path_data.Xpos[index], self.path_data.Ypos[index], self.path_data.Zpos[index]]).T
            tmp_pos = self.collision_pos[index, :]
            distance_s = 1E8

            for i in range(start_poses.shape[0]):
                # print("tmp_pos.shape", tmp_pos.shape)
                # print("start_poses", start_poses[i], "end_poses", end_poses[i])
                target = tmp_pos - start_poses[i] if (label+1) % 2 == 1 else tmp_pos - end_poses[i]
                Dis_S = np.linalg.norm(target, axis=1)
                min_Dis_s = np.min(Dis_S)
                # print("min_Dis_s", min_Dis_s)
                if min_Dis_s < distance_s:
                    distance_s = min_Dis_s
                    support_index[label] = i + 1

            target_idx = int(support_index[label] - 1)
            if direction:
                target_S = tmp_pos - start_poses[target_idx] if (label + 1) % 2 == 1 else tmp_pos - end_poses[target_idx]
                target_E = tmp_pos - end_poses[target_idx] if (label + 1) % 2 == 1 else tmp_pos - start_poses[target_idx]
            else:
                target_S = tmp_pos - start_poses[target_idx] if label% 2 == 1 else tmp_pos - end_poses[target_idx]
                target_E = tmp_pos - end_poses[target_idx] if label% 2 == 1 else tmp_pos - start_poses[target_idx]
            head_vec_main = np.array(
                [self.path_data.Xvec[target_idx], self.path_data.Yvec[target_idx], self.path_data.Zvec[target_idx]])
            head_vec_sub = np.array(
                [self.path_data.Xsubvec[target_idx], self.path_data.Ysubvec[target_idx], self.path_data.Zsubvec[target_idx]])
            # head_vec_normal = np.array(
            #     [self.path_data.Xnorm[target_idx], self.path_data.Ynorm[target_idx], self.path_data.Znorm[target_idx]])
            head_vec_main = head_vec_main / np.linalg.norm(head_vec_main)

            idx_max = np.argmin(np.linalg.norm(target_S))-1
            target_norm = target_S[idx_max] / np.linalg.norm(target_S[idx_max])
            flag = np.array(
                [np.dot(target_norm, head_vec_main), np.dot(target_norm, head_vec_sub)])
            flag_angle = np.rad2deg(np.arccos(flag[0]))
            print("flag_angle: ", flag_angle)
            # print("one dot", np.dot(target_norm, head_vec_main))
            if abs(flag_angle) < thresh_angle or abs(flag_angle-180) < thresh_angle:
                print("target_idx: ", target_idx)
                print("Find! at label: ", label + 1)
                print("min_Distance: ", distance_s, " mm")
                Dis_S = np.linalg.norm(target_S, axis=1)
                Dis_E = np.linalg.norm(target_E, axis=1)
                start_index[label] = index[0] + np.argmin(Dis_S)
                end_index[label] = index[0] + np.argmin(Dis_E)
                print("start_index: ", index[0] + np.argmin(Dis_S), "end_index: ", index[0] + np.argmin(Dis_E), "\n")
            else:
                # start_index[label] = np.append(start_index, index[0])
                # end_index[label] = np.append(end_index, index[0])
                print("index none ... ", "\n")

        # print("\nstart_index: \n", start_index.astype(int), "\nend_index: \n", end_index.astype(int), "\n")
        return start_poses, end_poses, start_index.astype(int), end_index.astype(int)

    def decide_simple_OnOff(self, direction=True):
        start_poses, end_poses, start_index, end_index = self.decide_start_pos_and_end_pos(direction)

        self.start_index = start_index
        self.end_index = end_index

        self.on_off = np.zeros(self.dataNum)
        self.delay_time = np.zeros(self.dataNum)
        for label in range(int(np.max(self.path_data.sub_label))):
            # index = np.where(self.path_data.sub_label == label + 1)[0]
            s_idx = start_index[label]
            e_idx = end_index[label]
            self.on_off[s_idx:e_idx] = 1
            self.delay_time[s_idx:e_idx] = 0


    def decide_delay_time(self):
        self.on_off = np.zeros(self.dataNum)
        self.delay_time = np.zeros(self.dataNum)
        search_index = 0
        for timing in self.pred_collision_time:
            # timing = self.pred_collision_time[index]
            search_target_array = self.time_array[search_index:]
            tmp = search_target_array - timing
            tmp[tmp>0] = -1 * float('inf')
            target_index = np.argmax(tmp)
            self.on_off[search_index + target_index] = 1
            self.delay_time[search_index + target_index] = -1 * np.max(tmp)
            search_index = target_index



