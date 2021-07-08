import numpy as np
from scipy.interpolate import interp1d
import re

import matplotlib.pyplot as plt


def CreateHeadNozzlesArray(Nozzle_XY, y_Head, x_diffs):
    x_Nozzle, y_Nozzle = Nozzle_XY[0], Nozzle_XY[1]
    Nozzles = np.zeros((8, 3))
    for i in range(8):
        Nozzles[i, 0] = -x_Nozzle * i
        Nozzles[i, 1] = (i % 2) * y_Nozzle

    # ヘッドのノズル配列の作成
    HeadNozzles = np.copy(Nozzles)
    # x_diffs = np.array([20.64, 22.33, 18.95, 21.49, 23.18, 19.79]) - 20.64
    for j in range(len(x_diffs)-1):
        tmp = np.copy(Nozzles)
        tmp[:, 0] = Nozzles[:, 0] + x_diffs[j + 1]
        tmp[:, 1] = Nozzles[:, 1] + y_Head * (j + 1)
        HeadNozzles = np.vstack((HeadNozzles, tmp))
    return HeadNozzles


def calcOrthogonal_Vector(a, b):
    ax, ay, az = a[:, 0], a[:, 1], a[:, 2]
    bx, by, bz = b[:, 0], b[:, 1], b[:, 2]
    c1 = [(-ay*bz + az*by)*np.sqrt(1/(ax**2*by**2 + ax**2*bz**2 - 2.0*ax*ay*bx*by - 2.0*ax*az*bx*bz + ay**2*bx**2 + ay**2*bz**2 - 2.0*ay*az*by*bz + az**2*bx**2 + az**2*by**2)),
          ( ax*bz - az*bx)*np.sqrt(1/(ax**2*by**2 + ax**2*bz**2 - 2.0*ax*ay*bx*by - 2.0*ax*az*bx*bz + ay**2*bx**2 + ay**2*bz**2 - 2.0*ay*az*by*bz + az**2*bx**2 + az**2*by**2)),
          (-ax*by + ay*bx)*np.sqrt(1/(ax**2*by**2 + ax**2*bz**2 - 2.0*ax*ay*bx*by - 2.0*ax*az*bx*bz + ay**2*bx**2 + ay**2*bz**2 - 2.0*ay*az*by*bz + az**2*bx**2 + az**2*by**2))]
    c2 = [( ay*bz - az*by)*np.sqrt(1/(ax**2*by**2 + ax**2*bz**2 - 2.0*ax*ay*bx*by - 2.0*ax*az*bx*bz + ay**2*bx**2 + ay**2*bz**2 - 2.0*ay*az*by*bz + az**2*bx**2 + az**2*by**2)),
          (-ax*bz + az*bx)*np.sqrt(1/(ax**2*by**2 + ax**2*bz**2 - 2.0*ax*ay*bx*by - 2.0*ax*az*bx*bz + ay**2*bx**2 + ay**2*bz**2 - 2.0*ay*az*by*bz + az**2*bx**2 + az**2*by**2)),
          ( ax*by - ay*bx)*np.sqrt(1/(ax**2*by**2 + ax**2*bz**2 - 2.0*ax*ay*bx*by - 2.0*ax*az*bx*bz + ay**2*bx**2 + ay**2*bz**2 - 2.0*ay*az*by*bz + az**2*bx**2 + az**2*by**2))]
    c1 = np.array(c1)
    c2 = np.array(c2)
    return [c1, c2]


def Sub_Read_StringRowData(filepath):
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


def Read_NCData(filepath, target_List=None, offset_rows=13):
    if target_List is None:
        target_List = ['N', 'X', 'Y', 'Z', 'A', 'F']
    str_data = Sub_Read_StringRowData(filepath)
    str_data[:offset_rows] = []

    tmp = [0, 0, 0, 0, 0, 0]
    i = 0
    data = []
    while True:  # str_dataのrow方向を確認するループ
        try:
            line = str_data[i].split()
            for jj in range(len(target_List)):  # target_listのマッチを確認するループ
                j = 0
                while True:  # str_dataのcolumn方向を確認するループ
                    try:
                        str = line[j]
                        tmp[jj] = float(str[1:]) if str[0] in target_List[jj] else tmp[jj]
                    except IndexError:  # column方向にデータがなくなったら終了
                        break
                    j += 1
            data.append(list(tmp))  # https://teratail.com/questions/21718
            i += 1
        except IndexError:  # row方向にデータがなくなったら終了
            break

    return(np.array(data))


def Archive_20210402_Sub_ReadRobotCode(filepath, file_num):
    # データ数が増えてtmp[:7]では対処できなくなったため，アーカイブ送り
    str_data = Sub_Read_StringRowData(filepath)
    i = -1
    data = []
    while True:
        i += 1
        try:
            tmp = str_data[i]
            if re.match(r"C[0-9]+=", tmp[:7]):
                data.append(list(map(float, tmp[6:].split(','))))
        except IndexError:  # 最後まで読み終わっている場合
            print('file No.', file_num, ' Complete !')
            break
    return np.array(data)


def Sub_ReadRobotCode(filepath, file_num):
    str_data = Sub_Read_StringRowData(filepath)
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


def Read_RobotCode(filelist):
    sFlag = True
    file_num = 1
    for file in filelist:
        if sFlag:
            data = Sub_ReadRobotCode(file, file_num)
            sFlag = False
        else:
            tmp = Sub_ReadRobotCode(file, file_num)
            data = np.vstack((data, tmp))
        file_num += 1
    return data


def calcDistance(array):
    distance = np.array([np.sqrt((array[i+1, 0]-array[i, 0])**2 + (array[i+1, 1]-array[i, 1])**2 + (array[i+1, 2]-array[i, 2])**2) for i in range(array.shape[0]-1)])
    return distance


def calcDistance_2D(array):
    distance = np.array([np.sqrt((array[i + 1, 0] - array[i, 0]) ** 2 + (array[i + 1, 1] - array[i, 1]) ** 2) for i in range(array.shape[0] - 1)])
    return distance


def shiftPos(Raw, Shift):
    R = np.copy(Raw)
    R[:, 0] += Shift[0]
    R[:, 1] += Shift[1]
    R[:, 2] += Shift[2]
    return R


def rotM(p):
    # https://org-technology.com/posts/rotational-transformation-matrix.html
    # 回転行列を計算する
    px = p[0] * (np.pi/180)
    py = p[1] * (np.pi/180)
    pz = p[2] * (np.pi/180)

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


# 塗装点データを副走査方向でラベリングするための関数など
def polar2cartesian(position):
    # https://moromi-senpy.hatenablog.com/entry/2019/08/06/174039
    num = position.shape[0]
    newPosition = np.empty([num, 3], dtype=np.float64)
    newPosition[:, 0] = position[:, 0] * np.sin(position[:, 1]) * np.cos(position[:, 2])
    newPosition[:, 1] = position[:, 0] * np.sin(position[:, 1]) * np.sin(position[:, 2])
    newPosition[:, 2] = position[:, 0] * np.cos(position[:, 1])
    # newPosition[:, 1] = newPosition[:, 1] * (360/np.pi)
    # newPosition[:, 2] = newPosition[:, 2] * (360/np.pi)
    return newPosition


def SpecifyVecChangePoint(array, thresh):
    # 極座標変換したベクトルデータを求める
    vecData = array[1:, :] - array[:-1, :]
    vecData = polar2cartesian(vecData)
    print('vec data', vecData)

    # plt.plot(vecData[:, 1])
    # plt.plot(vecData[:, 2])
    # plt.title('Check vec data theta')
    # plt.grid()
    # plt.show()


    # ベクトルデータで角度変化の大きい部分を抽出
    diff_vecData = abs( vecData[1:, 1:] - vecData[:-1, 1:])
    print('\ndiff', diff_vecData)

    # plt.plot(diff_vecData[:, 0])
    # plt.plot(diff_vecData[:, 1])
    # plt.grid()
    # plt.title('Check diff vec data change')
    # plt.show()

    tmp_flag = np.where((diff_vecData[:, 0] > thresh) | (diff_vecData[:, 1] > thresh), True, False)
    flag_index = np.array([True if tmp_flag[i] == True and tmp_flag[i+1] == True else False for i in range(tmp_flag.shape[0]-1)])
    flag_index = np.insert(flag_index, 0, tmp_flag[0])

    # Review
    # x = np.arange(diff_vecData.shape[0])
    plt.figure()
    # plt.scatter(x, abs(diff_vecData[:, 0]), marker='o', s=2)
    # plt.scatter(x, abs(diff_vecData[:, 1]), marker='o', s=2)
    plt.scatter(diff_vecData[:, 0], diff_vecData[:, 1], marker='o', s=3)
    plt.grid()
    plt.title('scatter plot')
    plt.show()

    return flag_index


def LabelingSubDirection(array, flag_index):
    indexes = np.where(flag_index == True)
    insertArray = np.zeros(flag_index.shape)
    labelNum = 1
    for item in indexes[0]:  # indexesの配列の形なんか特徴的
        item -= 1
        # print("item", item)
        if labelNum == 1:
            insertArray[:item] = labelNum
            labelNum += 1
        else:
            if abs(item - past_item) <= 2:
                insertArray[past_item:item] = labelNum + 1
            else:
                insertArray[past_item:item] = labelNum
                labelNum += 1
        past_item = item
    insertArray[item:] = labelNum  # 最後の処理が必要
    insertArray = np.insert(insertArray, 0, [1, 1])
    data = np.hstack((array, insertArray.reshape((insertArray.shape[0], 1))))
    return data


def Reverse_RefSubDirection(array, direction):
    #  副走査方向を参考に配列の方向を逆転
    """
    :param array: n行4列の配列 (0:X, 1:Y, 2:Z, 3: 副走査No)
    :param direction: forward方向とbackward方向どちらに合わせるか
    :return:
    """
    data = np.copy(array)
    for i in range(int(np.max(array[:, 3]))):
        if i % 2 == direction:
            index = np.where(array[:, 3] == i)
            data[index, :] = np.fliplr(array[index, :])
    return data


def CurveFitting3D_plusAlpha(input_data, ref_data, fix_space):
    # 線形補完した後で，固定幅で配列を作成
    fit_x = interp1d(ref_data, input_data[:, 0], kind='linear')
    fit_y = interp1d(ref_data, input_data[:, 1], kind='linear')
    fit_z = interp1d(ref_data, input_data[:, 2], kind='linear')
    fit_l = interp1d(ref_data, input_data[:, 3], kind='linear')
    space_data = np.arange(ref_data[0], ref_data[-1], fix_space)
    output = np.array([fit_x(space_data), fit_y(space_data), fit_z(space_data), fit_l(space_data)])
    return output.T


def Resampling_60dpi(data, fix_distance=0.4233):
    """
    :param data:n行4列の配列 (0:X, 1:Y, 2:Z, 3: 副走査No)
    :param fix_distance: resampling 間隔(60dpiの場合は，0.4233mm)
    :return: resampled_data: fix_distanceでリサンプリングされたdata
    """
    # 計算用配列の準備 (距離データの累積和を時間軸ととらえてリサンプルを行う)
    distance = calcDistance(data[:, :3])
    sum_distance = np.insert(np.cumsum(distance), 0, 0)  # 累積和で初項に0を代入
    print('Label Num Max: ', np.max(data[:, 3]))
    print('unique', np.unique(data[:, 3]))
    for label in range(int(np.max(data[:, 3]))):
        index = np.where(data[:, 3] == label+1)[0]
        target = data[index, :]
        time_data = sum_distance[index]
        time_data = time_data - time_data[0]

        tmp = CurveFitting3D_plusAlpha(target, time_data, fix_distance)

        if label == 0:
            resample_data = tmp
        else:
            resample_data = np.vstack((resample_data, tmp))
    return resample_data