# This is a sample Python script.
import math
from numpy import *
import numpy as np

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

DR = 0.017453292519943
RD = 57.295779513082323
PI = 3.141592653589793
ESP = 1e-12

FIX = 1

class kinematics:
    # DH参数
    # UR5
    # d = array([89.2, 0, 0, 109.3, 94.75, 82.5], dtype=np.float64)
    # a = array([0, 0, -425, -392, 0, 0], dtype=np.float64)
    # UR3
    d = FIX * array([151.9, 0, 0, 112.35, 85.35, 200 + 81.9], dtype=np.float64)
    a = FIX * array([0, 0, -243.65, -213.25, 0, 0], dtype=np.float64)
    alp = array([0, 90 * DR, 0, 0, 90 * DR, - 90 * DR], dtype=np.float64)
    th = array([0, 0, 0, 0, 0, 0], dtype=np.float64)  # 给定关节角度
    offset = array([0, -90 * DR, 0, 0, 0, 0], dtype=np.float64)  # arm处于竖直向上初始位置时的关节位置
    T06 = zeros((4, 4))
    q_sols = 0  # 逆解结果
    nums = 0  # 逆解组数
    q_opt = zeros(6)  # 最佳解
    q_last = zeros(6) # 上次解
    Tp = zeros((4, 4))  # 期望位姿矩阵
    rpy = zeros(3)  # 姿态
    pos = zeros(3)  # 姿态

    def test(self):
        nums = 8
        sums = zeros(nums)
        print(sums)
        sums[0] = 1.1
        print(sums[0])
        print(sums)

    def forward(self, th=zeros(6)):
        # 不带关节初始offset
        rst = eye(4)
        Ti = zeros((4, 4))
        self.th = th
        for i in range(6):
            Ti[0][0] = cos(self.th[i])
            Ti[0][1] = -sin(self.th[i])
            Ti[0][2] = 0
            Ti[0][3] = self.a[i]
            Ti[1][0] = sin(self.th[i]) * cos(self.alp[i])
            Ti[1][1] = cos(self.th[i]) * cos(self.alp[i])
            Ti[1][2] = -sin(self.alp[i])
            Ti[1][3] = -self.d[i] * sin(self.alp[i])
            Ti[2][0] = sin(self.th[i]) * sin(self.alp[i])
            Ti[2][1] = cos(self.th[i]) * sin(self.alp[i])
            Ti[2][2] = cos(self.alp[i])
            Ti[2][3] = cos(self.alp[i]) * self.d[i]
            Ti[3][0] = 0
            Ti[3][1] = 0
            Ti[3][2] = 0
            Ti[3][3] = 1
            # print(i, Ti)
            rst = dot(rst, Ti)
        self.T06 = rst
        # print(self.T06, '\n')

    def inverse(self, T_goal=eye(4)):
        self.nums=0
        # a0-a5; d1-d6; alp0-alp5; the1-the6
        # 将不为0的参数赋值
        a2 = self.a[2]
        a3 = self.a[3]
        d1 = self.d[0]
        d4 = self.d[3]
        d5 = self.d[4]
        # 计算invT67
        theta7 = 0
        a6 = 0
        afa6 = 0
        d7 = self.d[5]
        invT67 = array([[cos(theta7), -sin(theta7), 0, a6],
                        [sin(theta7) * cos(afa6), cos(theta7) * cos(afa6), -sin(afa6), -sin(afa6) * d7],
                        [sin(theta7) * sin(afa6), cos(theta7) * sin(afa6), cos(afa6), -cos(afa6) * d7],
                        [0, 0, 0, 1]], dtype=np.float64)
        T06 = dot(T_goal, invT67)
        # print(T06)
        # 将T06各元素提取出来赋值
        nx = T06[0][0]
        ny = T06[1][0]
        nz = T06[2][0]
        ox = T06[0][1]
        oy = T06[1][1]
        oz = T06[2][1]
        ax = T06[0][2]
        ay = T06[1][2]
        az = T06[2][2]
        px = T06[0][3]
        py = T06[1][3]
        pz = T06[2][3]
        # 初始化inv_theta矩阵
        inv_theta = zeros((8, 6))
        # S5 = array([0, 0, 0, 0], dtype=np.float64)
        # S234 = array([0, 0, 0, 0], dtype=np.float64)
        # C234 = array([0, 0, 0, 0], dtype=np.float64)
        # B1 = array([0, 0, 0, 0], dtype=np.float64)
        # B2 = array([0, 0, 0, 0], dtype=np.float64)
        # A = array([0, 0, 0, 0], dtype=np.float64)
        # B = array([0, 0, 0, 0], dtype=np.float64)
        # C = array([0, 0, 0, 0], dtype=np.float64)
        # theta1 = array([0, 0], dtype=np.float64)
        # theta2 = array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
        # theta3 = array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
        # theta4 = array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
        # theta5 = array([0, 0, 0, 0], dtype=np.float64)
        # theta6 = array([0, 0, 0, 0], dtype=np.float64)
        # theta23 = array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
        # theta234 = array([0, 0, 0, 0], dtype=np.float64)
        S5 = zeros(4)
        S234 = zeros(4)
        C234 = zeros(4)
        B1 = zeros(4)
        B2 = zeros(4)
        A = zeros(4)
        B = zeros(4)
        C = zeros(4)
        theta1 = zeros(2)
        theta2 = zeros(8)
        theta3 = zeros(8)
        theta4 = zeros(8)
        theta5 = zeros(4)
        theta6 = zeros(4)
        theta23 = zeros(8)
        theta234 = zeros(4)
        # 求解theta1
        space = px ** 2 + py ** 2 - d4 ** 2
        if space < -ESP:
            print('超出工作空间，无法求解')
            # self.q_sols = inv_theta
            return
        else:
            # 求解theta1
            if (space >= -ESP) and (space < 0):
                space = 0
        theta1[0] = math.atan2(py, px) - math.atan2(-d4, sqrt(space))
        theta1[1] = math.atan2(py, px) - math.atan2(-d4, -sqrt(space))
        # 求解theta5
        S5[0] = sqrt((-sin(theta1[0]) * nx + cos(theta1[0]) * ny) ** 2 + (-sin(theta1[0]) * ox + cos(theta1[0]) * oy) ** 2)
        theta5[0] = math.atan2(S5[0], sin(theta1[0]) * ax - cos(theta1[0]) * ay)
        S5[1] = -sqrt((-sin(theta1[0]) * nx + cos(theta1[0]) * ny) ** 2 + (-sin(theta1[0]) * ox + cos(theta1[0]) * oy) ** 2)
        theta5[1] = math.atan2(S5[1], sin(theta1[0]) * ax - cos(theta1[0]) * ay)
        S5[2] = sqrt((-sin(theta1[1]) * nx + cos(theta1[1]) * ny) ** 2 + (-sin(theta1[1]) * ox + cos(theta1[1]) * oy) ** 2)
        theta5[2] = math.atan2(S5[2], sin(theta1[1]) * ax - cos(theta1[1]) * ay)
        S5[3] = -sqrt((-sin(theta1[1]) * nx + cos(theta1[1]) * ny) ** 2 + (-sin(theta1[1]) * ox + cos(theta1[1]) * oy) ** 2)
        theta5[3] = math.atan2(S5[3], sin(theta1[1]) * ax - cos(theta1[1]) * ay)
        # 当S5不等于0时可求出如下theta6, 需要判断语句, 并求解theta234的和
        # 求解theta2
        # 求解theta23的和
        # 求解theta4 = theta234 - theta23
        # 求解theta3 = theta23 - theta2
        # 将解分成8组解，以弧度为单位
        for i in range(4):
            if i < 2:
                k = 0
            else:
                k = 1
            j = i * 2
            if fabs(S5[i]) > ESP:
                theta6[i] = math.atan2((-sin(theta1[k]) * ox + cos(theta1[k]) * oy) / S5[i], (sin(theta1[k]) * nx - cos(theta1[k]) * ny) / S5[i])
                S234[i] = -az / S5[i]
                C234[i] = -(cos(theta1[k]) * ax + sin(theta1[k]) * ay) / S5[i]
                theta234[i] = math.atan2(S234[i], C234[i])
                B1[i] = cos(theta1[k]) * px + sin(theta1[k]) * py - d5 * S234[i]
                B2[i] = pz - d1 + d5 * C234[i]
                A[i] = -2 * B2[i] * a2
                B[i] = 2 * B1[i] * a2
                C[i] = B1[i] ** 2 + B2[i] ** 2 + a2 ** 2 - a3 ** 2
                if A[i] ** 2 + B[i] ** 2 - C[i] ** 2 >= 0:
                    theta2[i] = math.atan2(B[i], A[i]) - math.atan2(C[i], sqrt(A[i] ** 2 + B[i] ** 2 - C[i] ** 2))
                    theta2[i + 1] = math.atan2(B[i], A[i]) - math.atan2(C[i], -sqrt(A[i] ** 2 + B[i] ** 2 - C[i] ** 2))
                    theta23[j] = math.atan2((B2[i] - a2 * sin(theta2[i])) / a3, (B1[i] - a2 * cos(theta2[i])) / a3)
                    theta23[j + 1] = math.atan2((B2[i] - a2 * sin(theta2[i + 1])) / a3, (B1[i] - a2 * cos(theta2[i + 1])) / a3)
                    theta4[i] = theta234[i] - theta23[j]
                    theta4[i + 1] = theta234[i] - theta23[j + 1]
                    theta3[i] = theta23[j] - theta2[i]
                    theta3[i + 1] = theta23[j + 1] - theta2[i + 1]
                    inv_theta[j] = array([theta1[k], theta2[i], theta3[i], theta4[i], theta5[i], theta6[i]])
                    inv_theta[j + 1] = array([theta1[k], theta2[i + 1], theta3[i + 1], theta4[i + 1], theta5[i], theta6[i]])
                    self.nums = self.nums + 2
        inv_theta = inv_theta[~(inv_theta == 0).all(1)]
        # print(inv_theta,self.nums)
        if self.nums > 0:
            for i in range(self.nums):
                for j in range(6):
                    if inv_theta[i][j] <= -pi:
                        inv_theta[i][j] = inv_theta[i][j] + 2 * pi
                    elif inv_theta[i][j] > pi:
                        inv_theta[i][j] = inv_theta[i][j] - 2 * pi
            # for i in range(self.nums - 1, -1, -1):
            #     if pi/4 < inv_theta[i][1] < pi:
            #         np.delete(inv_theta, i, axis=0)
            #         self.nums = self.nums - 1
        else:
            print("该位姿处于奇异位置有无穷解")
            return
        self.q_sols = inv_theta
        # print(self.q_sols)
        # print(self.nums)

    def getOptsolue(self, q, nums):
        if nums == 0 or q.any() == 0:
            return
        sums = array(zeros(nums))
        for i in range(nums):
            for j in range(6):
                sums[i] = sums[i] + abs(q[i][j]-self.q_last[j])
        small = sums[0]
        opt_index = 0
        for i in range(1, nums):
            if sums[i] < small:
                small = sums[i]
                opt_index = i
        self.q_opt  = q[opt_index]
        self.q_last = q[opt_index]


    def RPY2T(self):
        r = self.rpy[0]
        p = self.rpy[1]
        y = self.rpy[2]
        px = self.pos[0]
        py = self.pos[1]
        pz = self.pos[2]
        self.Tp = array(
            [[cos(p) * cos(y), cos(y) * sin(p) * sin(r) - cos(r) * sin(y), sin(r) * sin(y) + cos(r) * cos(y) * sin(p), px],
             [cos(p) * sin(y), cos(r) * cos(y) + sin(p) * sin(r) * sin(y), cos(r) * sin(p) * sin(y) - cos(y) * sin(r), py],
             [-sin(p), cos(p) * sin(r), cos(p) * cos(r), pz],
             [0, 0, 0, 1]])

    def T2RPY(self, T=eye(4)):
        if abs(T[2][0] - 1.0) < ESP:
            r = 0
            if T[2][0] < 0:
                y = -math.atan2(T[0][1], T[0][2])  # R - Y
            else:
                y = math.atan2(-T[0][1], -T[0][2])  # R + Y
            p = -math.asin(T[2][0])
        else:
            r = math.atan2(T[2][1], T[2][2])  # R
            y = math.atan2(T[1][0], T[0][0])  # Y
            p = math.atan2(-T[2][0] * cos(r), T[2][2])
        self.rpy = array([r, p, y], dtype=np.float64)

    # 设置rpy pos
    def setRP(self, rpy=zeros(3), pos=zeros(3)):
        self.rpy = rpy
        self.pos = pos

    # 由rpy pos 计算8组逆解,并选择最佳解
    def sloveQ(self):
        self.RPY2T()  # 计算self.Tp
        self.inverse(self.Tp)  # 计算self.q_sols、self.q_sols
        self.getOptsolue(self.q_sols, self.nums)  # 计算self.q_opt


def mytest():
    UR3 = kinematics()

    # theta = array([0, 0, 1.0, 0, 1.0, 0])  # 弧度
    # UR3.forward(theta)  # 正向
    # # print(UR3.T06)
    # UR3.inverse(UR3.T06)  # 逆向
    # #print(UR3.q_sols)
    # UR3.getOptsolue(UR3.q_sols, UR3.nums)  # 最优解
    # print(UR3.q_opt)


    rpy = array([1, 2, 3])
    pos = array([100, 100, 200])
    UR3.setRP(rpy, pos)
    UR3.sloveQ()
    print(UR3.Tp)
    print(UR3.q_opt)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    mytest()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
