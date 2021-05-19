from controller import Motor,DistanceSensor,Supervisor,GPS,Keyboard,Node,Gyro,Robot
import math
from numpy import *
import numpy as np
import arm
import time

PI = float(3.14159)
R2D = float(57.2956455) # 弧度转角度 =180/pi
D2R = float(0.01745333) # 角度转弧度 =pi/180

KEY_W = 87
KEY_S =	83
KEY_A =	65
KEY_D =	68
KEY_J =	74
KEY_K =	75
KEY_G =	71
KEY_H = 72

KEY_T = 84
KEY_Y = 89
KEY_U = 85
KEY_I = 73
KEY_O = 79
KEY_P = 80
KEY_J = 74
KEY_K = 75
KEY_L = 76

KEY_Q = 81
KEY_E = 69
KEY_F = 70

ALPH = 22.0
HALF_ALPH = 11.0

POS_RED_ORG = np.array([-3.79,0.86,4.81])
POS_BLUE_ORG = np.array([-3.55,0.86,4.81])
POS_GREEN_ORG = np.array([-3.28,0.86,4.81])
POS_BOX_ORG = np.array([-0.41,0.89,4.44])


POS_RED_TAG = np.array([-3.46,0.9,3.48])
POS_BLUE_TAG = np.array([-1.75,0.85,2.08])
POS_GREEN_TAG = np.array([-1.74,0.9,3.9])
POS_BOX_TAG = np.array([-0.34,0.7,-3.76])

class Robot1(object):
    """docstring for robot."""

    def __init__(self):
        super(Robot1, self).__init__()
        self.robot = Supervisor() # 初始化机器人节点
        self.timestep = int(self.robot.getBasicTimeStep())  #ms
        self.robotNode = self.robot.getFromDef('ODC05')

        # 开启键盘
        self.kb = Keyboard()
        self.kb.enable(self.timestep)
        # 开启GPS
        self.gps_grip = self.robot.getDevice('gps_grip')
        self.gps_base = self.robot.getDevice('gps_base')
        self.gps_grip.enable(self.timestep)
        self.gps_base.enable(self.timestep)
        # 开启陀螺仪
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timestep)

        self.w1 = Wheel()
        self.w2 = Wheel()
        self.w3 = Wheel()
        self.w4 = Wheel()

        self.h1 = Motor()
        self.h2 = Motor()
        self.h3 = Motor()

        self.sp = Motor() # shoulder_pan_joint
        self.sl = Motor() # shoulder_lift_joint
        self.eb = Motor() # elbow_joint
        self.wr1 = Motor() # wrist_1_joint
        self.wr2 = Motor() # wrist_2_joint
        self.wr3 = Motor() # wrist_3_joint

        self.get_id()
        self.enable_position_sensor()

        self.vel = 1
        self.sp.m_id.setVelocity(self.vel)  # [rad/s]
        self.sl.m_id.setVelocity(self.vel)
        self.eb.m_id.setVelocity(self.vel)
        self.wr1.m_id.setVelocity(self.vel)
        self.wr2.m_id.setVelocity(self.vel)
        self.wr3.m_id.setVelocity(self.vel)

        self.vx = 0.0
        self.vy = 0.0
        self.wo = 0.0

        self.grap_sig = 1   # 抓取标志位
        self.pos_sig = 1    # 正姿标志位
        self.counter = 0
        self.counter_arm = 0

        self.body_width = 0.192  # 车身宽度
        self.body_length = 0.260  # 车身长度
        self.wheel_rad = 0.160    # 轮子半径

        # arm
        self.UR3 = arm.kinematics()
        self.pos_arm = 0
        self.rot_arm = 0
        self.ARoffset = np.array([0 ,0.17 , 0.23])  # arm与robot的位置修正
        # self.ARoffset = np.array([0 ,0.17 ,0.15])


        print("robot inited!\n")

    def get_id(self):
        self.w1.in_motor.m_id = self.robot.getDevice('mWheel1a')
        self.w2.in_motor.m_id = self.robot.getDevice('mWheel2a')
        self.w3.in_motor.m_id = self.robot.getDevice('mWheel3a')
        self.w4.in_motor.m_id = self.robot.getDevice('mWheel4a')

        self.w1.out_motor.m_id = self.robot.getDevice('mWheel1b')
        self.w2.out_motor.m_id = self.robot.getDevice('mWheel2b')
        self.w3.out_motor.m_id = self.robot.getDevice('mWheel3b')
        self.w4.out_motor.m_id = self.robot.getDevice('mWheel4b')

        self.w1.in_motor.p_id = self.robot.getDevice('pWheel1a')
        self.w2.in_motor.p_id = self.robot.getDevice('pWheel2a')
        self.w3.in_motor.p_id = self.robot.getDevice('pWheel3a')
        self.w4.in_motor.p_id = self.robot.getDevice('pWheel4a')

        self.w1.out_motor.p_id = self.robot.getDevice('pWheel1b')
        self.w2.out_motor.p_id = self.robot.getDevice('pWheel2b')
        self.w3.out_motor.p_id = self.robot.getDevice('pWheel3b')
        self.w4.out_motor.p_id = self.robot.getDevice('pWheel4b')

        self.h1.m_id = self.robot.getDevice('finger_1_joint_1')
        self.h2.m_id = self.robot.getDevice('finger_2_joint_1')
        self.h3.m_id = self.robot.getDevice('finger_middle_joint_1')

        self.sp.m_id = self.robot.getDevice('shoulder_pan_joint')
        self.sl.m_id = self.robot.getDevice('shoulder_lift_joint')
        self.eb.m_id = self.robot.getDevice('elbow_joint')
        self.wr1.m_id = self.robot.getDevice('wrist_1_joint')
        self.wr2.m_id = self.robot.getDevice('wrist_2_joint')
        self.wr3.m_id = self.robot.getDevice('wrist_3_joint')

    def enable_position_sensor(self):
        self.w1.in_motor.p_id.enable(self.timestep)
        self.w2.in_motor.p_id.enable(self.timestep)
        self.w3.in_motor.p_id.enable(self.timestep)
        self.w4.in_motor.p_id.enable(self.timestep)

        self.w1.out_motor.p_id.enable(self.timestep)
        self.w2.out_motor.p_id.enable(self.timestep)
        self.w3.out_motor.p_id.enable(self.timestep)
        self.w4.out_motor.p_id.enable(self.timestep)

    def robot_double_star_track(self):
        self.w1.double_star_track()
        self.w2.double_star_track()
        self.w3.double_star_track()
        self.w4.double_star_track()

    def robot_refresh_motor_position(self):
        self.w1.refresh_wheel_position()
        self.w2.refresh_wheel_position()
        self.w3.refresh_wheel_position()
        self.w4.refresh_wheel_position()

    def mecanum_kinematic(self):
        self.w1.in_motor.speed    = -self.vx + self.vy - (self.body_width + self.body_length) * self.wo;
        self.w1.out_motor.speed = -self.vx + self.vy - (self.body_width + self.body_length) * self.wo;
        self.w2.in_motor.speed   =  self.vx + self.vy - (self.body_width + self.body_length) * self.wo;
        self.w2.out_motor.speed =  self.vx + self.vy - (self.body_width + self.body_length) * self.wo;
        self.w3.in_motor.speed   = -self.vx + self.vy + (self.body_width + self.body_length) * self.wo;
        self.w3.out_motor.speed = -self.vx + self.vy + (self.body_width + self.body_length) * self.wo;
        self.w4.in_motor.speed   =  self.vx + self.vy + (self.body_width + self.body_length) * self.wo;
        self.w4.out_motor.speed = self.vx + self.vy + (self.body_width + self.body_length) * self.wo;

    def set_vx_vy_wo(self,vx: float,vy: float,wo: float):
         self.vx = vx
         self.vy = vy
         self.wo = wo

    def ctrl_key_scan(self):
        key = self.kb.getKey()
        if key == KEY_W:
            self.set_vx_vy_wo(0.0,0.03,0.0)
        elif key == KEY_S:
            self.set_vx_vy_wo(0.0,-0.03,0.0)
        elif key == KEY_A:
            self.set_vx_vy_wo(0.0,0.0,0.05)
        elif key == KEY_D:
            self.set_vx_vy_wo(0.0,0.0,-0.05)
        elif key == KEY_Q:
            self.set_vx_vy_wo(0.02,0.0,0.0)
        elif key == KEY_E:
            self.set_vx_vy_wo(-0.02,0.0,0.0)
        elif key == KEY_F:
            self.robot.step(100)
            if key == KEY_F:
                self.hand_grap()
        elif key == KEY_T:
            self.robot.step(100)
            if key == KEY_T:
                self.hand_release()
        # 取RGB物品
        elif key == KEY_Y:
            self.tag_get(KEY_Y,POS_RED_ORG)
        elif key == KEY_U:
            self.tag_get(KEY_U,POS_BLUE_ORG)
        elif key == KEY_I:
            self.tag_get(KEY_I,POS_GREEN_ORG)
        # 取盒子
        elif key == KEY_O:
            self.tag_get(KEY_O,POS_BOX_ORG)
        # 放RGB物品
        elif key == KEY_G:
            self.tag_get(KEY_G,POS_RED_TAG)
        elif key == KEY_H:
            self.tag_get(KEY_H,POS_BLUE_TAG)
        elif key == KEY_J:
            self.tag_get(KEY_J,POS_GREEN_TAG)
        # 放盒子
        elif key ==KEY_K:
            self.tag_get(KEY_K,POS_BOX_TAG)
        # 下抓
        elif key == KEY_P:
            self.robot.step(100)
            if key == KEY_P:
                if self.pos_sig == 1:
                    self.test_downcatch()
                    self.pos_sig = 0
                else:
                    self.init_arm_pos()
                    self.pos_sig = 1
        else:
            self.set_vx_vy_wo(0.0,0.0,0.0)
            # self.hand_release()

    def hand_grap(self):
        self.h1.m_id.setPosition(0.85)
        self.h2.m_id.setPosition(0.85)
        self.h3.m_id.setPosition(0.85)

    def hand_release(self):
        self.h1.m_id.setPosition(self.h1.m_id.getMinPosition())
        self.h2.m_id.setPosition(self.h2.m_id.getMinPosition())
        self.h3.m_id.setPosition(self.h3.m_id.getMinPosition())

    def get_gps(self):
        return np.array(self.gps_base.getValues())

    def move(self):

        self.robot_double_star_track()
        self.robot_refresh_motor_position()
        self.mecanum_kinematic()

    # 机械臂到达目标位置
    def arm_reach(self, obj_pos ,KEY,rob_rpy = array([ -pi/2, 0, -pi])):
        # rc_pos = np.array([-3790,550,4810])
        # pos = rc_pos - self.get_gps()

        pos = self.cal_obj_pos_robot(obj_pos)
        pos = self.WebotsPos2UR5Pos(pos)

        print(f'pos is {pos}')
        rpy  = rob_rpy

        # # 关节空间规划
        posg = np.copy(pos)
        posg[2] = posg[2] + 50
        print('pos0=',posg,'pos=',pos)

        q0 = np.copy(self.UR3.q_opt)
        self.UR3.setRP(rpy, pos)
        self.UR3.sloveQ()
        dq=self.UR3.q_opt - q0
        NS = 50
        for i in range(NS):
            self.robot.step(1)
            Pq = q0 + i*dq/NS
            self.UR3.q_opt = np.copy(Pq)
            self.sp.m_id.setPosition(self.UR3.q_opt[0])
            self.sl.m_id.setPosition(self.UR3.q_opt[1])
            self.eb.m_id.setPosition(self.UR3.q_opt[2])
            self.wr1.m_id.setPosition(self.UR3.q_opt[3]) #[-4.080976820730941, 3.633675958610736, 3.865641923274617]
            self.wr2.m_id.setPosition(self.UR3.q_opt[4])
            self.wr3.m_id.setPosition(self.UR3.q_opt[5])
        self.robot.step(100)
        print(self.UR3.nums)
        if self.UR3.nums>0:
            if KEY == KEY_U or KEY == KEY_I or KEY == KEY_Y or KEY == KEY_O:
                self.hand_grap()
                self.robot.step(100)
                self.init_arm_pos()
            elif KEY == KEY_G or KEY == KEY_H or KEY == KEY_J or KEY == KEY_K:
                self.hand_release()
                self.set_vx_vy_wo(0.0,-0.03,0.0)
                NS = 10
                for i in range(NS):
                    self.robot.step(1)
                    self.move()
                self.set_vx_vy_wo(0.0,0,0.0)
                self.init_arm_pos()

    def tag_get(self,KEY,TAG):
        self.robot.step(20)
        key = self.kb.getKey()
        self.arm_reach(TAG,KEY)
        self.pos_sig = 0

    # 下抓
    def test_downcatch(self):
        pos = array([0,-300.0, -270.0])
        rpy = array([ -pi, 0, 0])
        q0 = np.copy(self.UR3.q_opt)
        # self.UR3.setRP(rpy, pos)
        # self.UR3.sloveQ()
        print(self.UR3.q_opt)
        qd = array([ 1.18693885 ,-0.14274737  ,1.10250046  ,0.61104324  ,1.57079633 ,-0.38385747])
        dq= qd - q0
        NS = 50
        for i in range(NS):
            self.robot.step(1)
            Pq = q0 + i*dq/NS
            self.UR3.q_opt = np.copy(Pq)
            self.sp.m_id.setPosition(self.UR3.q_opt[0])
            self.sl.m_id.setPosition(self.UR3.q_opt[1])
            self.eb.m_id.setPosition(self.UR3.q_opt[2])
            self.wr1.m_id.setPosition(self.UR3.q_opt[3]) #[-4.080976820730941, 3.633675958610736, 3.865641923274617]
            self.wr2.m_id.setPosition(self.UR3.q_opt[4])
            self.wr3.m_id.setPosition(self.UR3.q_opt[5])

    # 机械臂正姿
    def init_arm_pos(self):
        i = 1
        if i == 1:

            rpy = array([ 0, 0, 0])
            # # pos = array([ -8.53713254e+01, -1.94250000e+02, 6.08799654e+02])
            # pos = array([ -85, -364, 600])   # [ x, y , z] [左+,后+，上+]
            pos = array([ 0, 0, 0 ])   # [ x, y , z] [左+,后+，上+]
            # self.UR3.setRP(rpy, pos)
            # self.UR3.sloveQ()

            # 正立姿态
            q0 = np.copy(self.UR3.q_opt)
            self.UR3.q_opt = array([0,-pi/2,0,-pi/2,0, 0])
            self.UR3.forward(self.UR3.q_opt)
            self.UR3.T2RPY(self.UR3.T06)

            pos[0]=self.UR3.T06[0][3]
            pos[1]=self.UR3.T06[1][3]
            pos[2]=self.UR3.T06[2][3]
            self.UR3.setRP(self.UR3.rpy, pos)
            # print(f'rpy:{self.UR3.rpy}')
            # print(f'pos:{self.UR3.pos}')
            dq=self.UR3.q_opt-q0
            NS = 50
            for i in range(NS):
                self.robot.step(1)
                Pq = q0 + i*dq/NS
                self.UR3.q_opt = np.copy(Pq)
            # print(self.UR3.q_sols)
            # print(self.UR3.q_opt)
            # print(self.UR3.Tp)
                self.sp.m_id.setPosition(self.UR3.q_opt[0])
                self.sl.m_id.setPosition(self.UR3.q_opt[1])
                self.eb.m_id.setPosition(self.UR3.q_opt[2])
                self.wr1.m_id.setPosition(self.UR3.q_opt[3]) #[-4.080976820730941, 3.633675958610736, 3.865641923274617]
                self.wr2.m_id.setPosition(self.UR3.q_opt[4])
                self.wr3.m_id.setPosition(self.UR3.q_opt[5])

        elif i == 2:
            theta = array([0, 0, arm.ESP, 0, arm.ESP, 0])  # 弧度
            theta = theta + self.UR3.offset
            self.UR3.forward(theta)  # 正向
            print("T06：\n",self.UR3.T06)
            self.UR3.T2RPY(self.UR3.T06)
            print("rpy：\n",self.UR3.rpy)
            self.UR3.inverse(self.UR3.T06)  # 逆向
            # print(self.UR3.q_sols)
            self.UR3.getOptsolue(self.UR3.q_sols, self.UR3.nums)  # 最优解
            # print(self.UR3.q_opt)
            self.sp.m_id.setPosition(self.UR3.q_opt[0])
            self.sl.m_id.setPosition(self.UR3.q_opt[1])
            self.eb.m_id.setPosition(self.UR3.q_opt[2])
            self.wr1.m_id.setPosition(self.UR3.q_opt[3])
            self.wr2.m_id.setPosition(self.UR3.q_opt[4])
            self.wr3.m_id.setPosition(self.UR3.q_opt[5])

    # 计算obj在robot坐标系下位置
    def cal_obj_pos_robot(self, obj_pos_world):
        self.rot_arm = np.array(self.robotNode.getOrientation())   # 获取方位角矩阵
        self.rot_arm = self.rot_arm.reshape(3, 3)   # 1*9 矩阵转换维 3*3
        self.rot_arm = np.transpose(self.rot_arm)   # 单位阵转置 = 求逆
        self.pos_arm = self.get_gps()   # 获取arm的位置，ARoffset是robot与arm的位置修正值
        #self.pos_arm = np.array(self.robotNode.getPosition()) + self.ARoffset   # 获取arm的位置，ARoffset是robot与arm的位置修正值
        obj_pos_world = np.subtract(obj_pos_world, self.pos_arm)    # robot与obj相减获取相对位置
        obj_pos_robot = np.dot(self.rot_arm, obj_pos_world)    # 转换obj坐标位置到robot坐标系中
        return 1000 * obj_pos_robot # m 转mm 单位

    # 转换webots下的位置数据 --> ur5要求的数据位置
    def WebotsPos2UR5Pos(self, pos):
        temp = pos[1]
        pos[1] = -pos[2]
        pos[2] = temp
        pos[1] = pos[1]
        return pos


class Wheel(object):
    """docstring for wheel."""

    def __init__(self):
        super(Wheel, self).__init__()
        self.in_motor = Motor()
        self.out_motor = Motor()

    def refresh_wheel_position(self):
        self.in_motor.refresh_motor_position()
        self.out_motor.refresh_motor_position()

    def double_star_track(self):
        if (math.fabs(self.in_motor.position_180) >= 180 - HALF_ALPH):
            self.in_motor.delt_position = self.in_motor.speed
        elif (math.fabs(self.in_motor.position_180) < HALF_ALPH):
            self.in_motor.delt_position = self.in_motor.speed
        else:
            self.in_motor.delt_position = ((90 - HALF_ALPH) / HALF_ALPH) * self.in_motor.speed
        self.refresh_out_motor_position()

    def refresh_out_motor_position(self):
        self.out_motor.delt_position = 0.0
        n = 0
        pm = 0
        if (self.in_motor.position >= 0):
            n = int(self.in_motor.position * R2D / 180)
            pm = 1
        else:
            n = int(-(- self.in_motor.position * R2D / 180))
            pm = -1
        # print(n)
        x = self.in_motor.position_180
        if (math.fabs(x) < HALF_ALPH):
            self.out_motor.position = D2R * ( n * 180 + ((90 - HALF_ALPH) / HALF_ALPH) * x)
        elif (math.fabs(x) > (180 - HALF_ALPH)):
            self.out_motor.position = D2R * (n * 180 + (180 - ALPH) / ALPH * x + pm * (  180 - (180 - ALPH) / ALPH * 180  )  )
        else:
            self.out_motor.position = D2R * (n * 180 + ALPH / (180 - ALPH) * x + pm * (  90 - HALF_ALPH - ALPH * ALPH / (2 * (180 - ALPH))  )  )

class Motor(object):
    """docstring for motor."""

    def __init__(self):
        super(Motor, self).__init__()
        self.speed = 0
        self.position = 0
        self.delt_position = 0
        self.position_180 = 0
        # self.m_id1 = motorname
        # self.p_id1 = motorname.replace('m', 'p')
        self.m_id = ''
        self.p_id = ''

    def refresh_motor_position(self):
        self.position += self.delt_position;
        self.m_id.setPosition(self.position)
        self.m_id.setAvailableTorque(200)
        self.position_180 = math.fmod(R2D * self.position, 180)




# from controller import Robot,Motor,DistanceSensor,Supervisor,GPS
#
# class Motor(Wheel):
#     """docstring for motor."""
#
#     def __init__(self, motorname: 'str'):
#         super(Motor, self).__init__()
#         self.speed = 0
#         self.position = 0
#         self.delt_position = 0
#         self.position_180 = 0
#         self.m_id = motorname
#         self.p_id = motorname.replace('m', 'p')
#         self.m_id = Robot.robot.getDevice(MotorName)   # 获取电机ID
#         self.m_id = Robot.robot.getDevice(MotorName.replace('m', 'p')) #获取编码器ID
#
#
# class Wheel(Robot):
#     """docstring for wheel."""
#
#     def __init__(self, in_motor: 'str', out_motor: 'str'):
#         super(Wheel, self).__init__()
#         self.in_motor = Motor(in_motor)
#         self.out_motor = Motor(out_motor)
#
#
# class Robot(object):
#     """docstring for robot."""
#
#     def __init__(self):
#         super(Robot, self).__init__()
#         self.robot = Supervisor() # 初始化机器人节点
#         self.timestep = int(self.robot.getBasicTimeStep())  #ms
#
#         self.w1 = Wheel('mWheel1a', 'mWheel1b')
#         self.w2 = Wheel('mWheel2a', 'mWheel2b')
#         self.w3 = Wheel('mWheel3a', 'mWheel3b')
#         self.w4 = Wheel('mWheel4a', 'mWheel4b')
#
#         self.vx = 0.0
#         self.vy = 0.0
#         self.wo = 0.0
#
#         self.body_wide = 0.0    # 车身宽度
#         self.body_length = 0.0  # 车身宽度
#         self.wheel_rad = 0.0    # 车身宽度
