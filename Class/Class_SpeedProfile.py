from Class_BasicData import BasicData
import numpy as np
from scipy.interpolate import interp1d


class SpeedProfile(BasicData):
    def __init__(self):
        print("\n-- initialize -- @SpeedProfile")
        super(SpeedProfile, self).__init__()
        self.raw_dataNum = 0
        self.raw_time_step = 0
        self.raw_time = np.array([])
        self.raw_Speed = np.array([])

        self.re_dataNum = 0
        self.re_time_step = 0
        self.re_time = np.array([])
        self.re_Speed = np.array([])
        self.re_distance = np.array([])

    def read_datafile(self, filepath):
        print("-- call read_datafile -- @SpeedProfile")
        tmp_data = np.loadtxt(filepath, delimiter=',', skiprows=1, encoding='utf-8')
        self.raw_dataNum = tmp_data.shape[0]
        self.raw_time = tmp_data[:, 0]
        self.raw_Speed = tmp_data[:, 1]
        self.raw_time_step = tmp_data[1, 0] - tmp_data[0, 0]

    def read_from_csv(self, filepath, skip_rows=7):
        tmp_data = np.loadtxt(filepath, delimiter=',', skiprows=skip_rows, encoding='utf-8')
        self.dataNum = tmp_data.shape[0]
        self.raw_time_step = tmp_data[1, 0] - tmp_data[0, 0]
        self.time = tmp_data[:, 0]
        self.Xpos = tmp_data[:, 1]
        self.Ypos = tmp_data[:, 2]
        self.Zpos = tmp_data[:, 3]
        self.Xangle = tmp_data[:, 4]
        self.Yangle = tmp_data[:, 5]
        self.Zangle = tmp_data[:, 6]
        self.Speed = tmp_data[:, 7]
        self.calc_mainvector()

    def resampling_data(self, delta_t):
        print("-- call resampling_data -- @SpeedProfile")
        self.re_time_step = delta_t
        fit_speed = interp1d(self.raw_time, self.raw_Speed, kind='linear')
        self.re_time = np.arange(self.raw_time[0], self.raw_time[-1]+self.re_time_step, self.re_time_step)
        self.re_Speed = np.array(fit_speed(self.re_time))
        self.re_dataNum = self.re_Speed.shape[0]

    def read_all_datafile(self, filepath):
        print("-- call read_datafile -- @SpeedProfile")
        tmp_data = np.loadtxt(filepath, delimiter=',', skiprows=1, encoding='utf-8')
        self.raw_dataNum = tmp_data.shape[0]
        self.raw_time = tmp_data[:, 0]
        self.raw_Speed = tmp_data[:, 1]
        self.raw_time_step = tmp_data[1, 0] - tmp_data[0, 0]

    def resampling_all_data(self, new_instance, re_delta_t):
        print("-- call resampling_data -- @SpeedProfile")
        self.re_time_step = re_delta_t

        fit_speed = interp1d(self.time, self.Speed, kind='linear')
        fit_xpos = interp1d(self.time, self.Xpos, kind='linear')
        fit_ypos = interp1d(self.time, self.Ypos, kind='linear')
        fit_zpos = interp1d(self.time, self.Zpos, kind='linear')
        fit_xangle = interp1d(self.time, self.Xangle, kind='linear')
        fit_yangle = interp1d(self.time, self.Yangle, kind='linear')
        fit_zangle = interp1d(self.time, self.Zangle, kind='linear')

        new_instance.time = np.arange(self.time[0], self.time[-1], self.re_time_step)
        new_instance.Speed = np.array(fit_speed(new_instance.time))
        new_instance.Xpos = np.array(fit_xpos(new_instance.time))
        new_instance.Ypos = np.array(fit_ypos(new_instance.time))
        new_instance.Zpos = np.array(fit_zpos(new_instance.time))
        new_instance.Xangle = np.array(fit_xangle(new_instance.time))
        new_instance.Yangle = np.array(fit_yangle(new_instance.time))
        new_instance.Zangle = np.array(fit_zangle(new_instance.time))
        new_instance.dataNum = new_instance.Speed.shape[0]
        new_instance.calc_mainvector()
        new_instance.calc_cumsum_distance()
        return new_instance


