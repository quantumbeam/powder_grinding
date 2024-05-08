import sys
sys.path.append("pybcapclient")

import pybcapclient.bcapclient as bcapclient
import time
import math

class CobottaController:
    HOST = "192.168.56.11"
    PORT = 5007
    TIMEOUT = 2000
    COMP = 1
    FLOOR_HEIGHT = 20
    TOWER_DELTA_Y = 50
    fixed_positions={
        "sleep":[290,-44.5,17,-90,0,-90,5],
        "in_front_of_platform":[21.07,277.54,110.38,-90.06,0.01,0.22,5.0],
        "above_platform":[21.07,335,107.38,-89.5,-0.6,0.22,5.0], #[21.07,335,107.38,-89.5,-1.6,0.22,5.0], #[21.07,331.88,110.38,-89.5,-1.6,0.22,5.0],
        "before_miniflex":[294.92,96.04,252.55,-90,0,-73,5.0],
        "before_funnel_drawer":[84.7, 333.18, 173.61, 153.2, -81.95, 133.04, 5.0], #[77.85,316.74,172.82,-119.65,-82.52,32.76,5.0],
        "before_platform":[21.07,277.54,110.38,-90.06,0.01,0.22,5.0],
        "before_closing_funnel":[118.94, 303.67, 142.74, -119.71, -82.49, 25.24, 5.0], #[118.89,303.72,151.81,-119.67,-82.51,25.21,5.0],
        "in_miniflex":[
            [268,-49,410,-90,0,-90,261],
            [-50,-268,411,-90,0,-180,261],
            [-61.4,-327.8,382.4,-90,0,-180,261],
            [-11.67, -331.60, 382.2, -89.02, -1.78, -167.78, 261], #[-11.25,-333.26,382.43,-90,-0.01,-170.08,261],
            [-11.67, -331.60, 377.2, -89.02, -1.78, -167.78, 261], #[-10.9,-333.29,377.61,-89.99,-0.01,-169.98,261],
            [-10.81,-333.29,388.98,-89.99,-0.002,-169.99,261],
            ],
        "center_of_platform":[13.61,336.37,152.51,-89.07,-1.39,-1.23,5.0], #[13.65,331.54,154.39,-90.06,0.01,0.22,5.0],
        "mount_center":[13.7, 336.31, 146.12, -89.19, -1.34, -1.24, 5.0],
        "before_mounting":[13.65,277.54,154.39,-90.06,0.01,0.22,5.0],
        }
    
    # 5mm shift
    variable_positions={
        "hotel_0-0":[360.90, 5.45, 32.17, -89.45, -1.59, -90.05, 5.0],
        "platform_stage":[21.12, 334.92, 105.4, -89.66, -0.58, 0.2, 5.0],
        "miniflex_5":[-9.94, -333.04, 381.43, -88.93, -0.94, -171.17, 261.0]
        }

    def __init__(self, name="", provider="CaoProv.DENSO.VRC", machine="localhost", option=""):
        self.name = name
        self.provider = provider
        self.machine = machine
        self.option = option
        self.m_bcapclient = None
        self.hCtrl = None
        self.HRobot = None


    ### general functions ###
    def connect(self):
        """
        COBOTTAに接続する
        """
        self.m_bcapclient = bcapclient.BCAPClient(CobottaController.HOST, CobottaController.PORT, CobottaController.TIMEOUT)
        self.m_bcapclient.service_start("")
        self.hCtrl = self.m_bcapclient.controller_connect(self.name, self.provider, self.machine, self.option)
        self.HRobot = self.m_bcapclient.controller_getrobot(self.hCtrl, "Arm", "")

        Command = "TakeArm"
        Param = [0, 0]
        self.m_bcapclient.robot_execute(self.HRobot, Command, Param)

        Command = "Motor"
        Param = [1, 0]
        self.m_bcapclient.robot_execute(self.HRobot, Command, Param)

        Command = "ExtSpeed"
        Speed = 100
        Accel = 100
        Decel = 100
        Param = [Speed, Accel, Decel]
        self.m_bcapclient.robot_execute(self.HRobot, Command, Param)

        IOHandl = self.m_bcapclient.controller_getvariable(self.hCtrl, "IO25", "")
        self.m_bcapclient.variable_putvalue(IOHandl, True)
        if(IOHandl != 0):
            self.m_bcapclient.variable_release(IOHandl)
        IOHandl = self.m_bcapclient.controller_getvariable(self.hCtrl, "IO26", "")
        self.m_bcapclient.variable_putvalue(IOHandl, True)
        if(IOHandl != 0):
            self.m_bcapclient.variable_release(IOHandl)
        self.m_bcapclient.service_stop()
        print('Connected')

    def disconnect(self):
        """
        COBOTTAとの接続を切断する
        """
        Command = "Motor"
        Param = [0,0]
        self.m_bcapclient.robot_execute(self.HRobot,Command,Param)
        ###Give Arm
        Command = "GiveArm"
        Param = None
        self.m_bcapclient.robot_execute(self.HRobot,Command,Param)
        #Disconnect
        if(self.HRobot != 0):
            self.m_bcapclient.robot_release(self.HRobot)
        #End If
        if(self.hCtrl != 0):
            self.m_bcapclient.controller_disconnect(self.hCtrl)
        #End If
        self.m_bcapclient.service_stop()
        print('Disconnected')

    def auto_calibration(self):
        """
        COBOTTAのキャリブレーションを行う

        終了後はホームポジションに移動し、モーターをオフにする
        """
        try:
            self.disconnect()
        except:
            pass
        # Connection processing of tcp communication
        m_bcapclient = bcapclient.BCAPClient(CobottaController.HOST, CobottaController.PORT, CobottaController.TIMEOUT)

        # start b_cap Service
        m_bcapclient.service_start("")

        # set Parameter
        Name = ""
        Provider = "CaoProv.DENSO.VRC"
        Machine = "localhost"
        Option = ""

        try:
            # Connect to RC8 (RC8(VRC)provider) , Get Controller Handle
            hCtrl = m_bcapclient.controller_connect(Name, Provider, Machine, Option)
            print("Connect RC8")
            # Get Robot Handle
            HRobot = m_bcapclient.controller_getrobot(hCtrl, "Arm", "")

            m_bcapclient.robot_execute(HRobot, "AutoCal", "")
            m_bcapclient.robot_execute(HRobot, "MotionPreparation", "")
            m_bcapclient.robot_execute(HRobot, "TakeArm")
            m_bcapclient.robot_execute(HRobot, "ExtSpeed", 100)
            m_bcapclient.robot_move(HRobot, 1, "@P J(0,0,90,0,90,0)")
            time.sleep(1)
            Command = "Motor"
            Param = [0,0]
            m_bcapclient.robot_execute(HRobot,Command,Param)    
        except Exception as e:
            print('=== ERROR Description ===')
            if str(type(e)) == "<class 'pybcapclient.orinexception.ORiNException'>":
                print(e)
                errorcode_int = int(str(e))
                if errorcode_int < 0:
                    errorcode_hex = format(errorcode_int & 0xffffffff, 'x')
                else:
                    errorcode_hex = hex(errorcode_int)
                print("Error Code : 0x" + str(errorcode_hex))
                error_description = m_bcapclient.controller_execute(
                    hCtrl, "GetErrorDescription", errorcode_int)
                print("Error Description : " + error_description)
            else:
                print(e)

        finally:
            # DisConnect
            if(HRobot != 0):
                m_bcapclient.robot_release(HRobot)
            # End If
            if(hCtrl != 0):
                m_bcapclient.controller_disconnect(hCtrl)
            # End If
            m_bcapclient.service_stop()
        print("AutoCalibration finished")

    def change_speed(self,speed=100,accel=100,decel=100):
        """
        COBOTTAの速度を変更する
        
        Parameters
        ----------
        speed : int, optional
            速度, by default 100, 0~100
        accel : int, optional
            加速度, by default 100, 0~100
        decel : int, optional
            減速度, by default 100, 0~100
        """
        Command = "ExtSpeed"
        Param = [speed,accel,decel]
        self.m_bcapclient.robot_execute(self.HRobot,Command,Param)
        print("Speed parameters changed")
        print(f"Speed: {speed}, Accel: {accel}, Decel: {decel}")

    def get_now_position(self):
        """
        現在の位置(P型)を取得する

        Returns
        -------
        list
            現在の位置(P型) [x,y,z,rx,ry,rz,fig]
        """
        return self.m_bcapclient.robot_execute(self.HRobot, "CurPos")

    def get_now_joint(self):
        """
        現在の関節角(J型)を取得する

        Returns
        -------
        list
            現在の関節角(J型) [j1,j2,j3,j4,j5,j6]
        """
        return self.m_bcapclient.robot_execute(self.HRobot, "CurJnt")

    def clear_error(self):
        """
        エラーを解除する
        """
        self.m_bcapclient.controller_execute(self.hCtrl,"ManualResetPreparation","")
        self.m_bcapclient.controller_execute(self.hCtrl,"ClearError","")
        print("Error cleared")

    def motion_preparation(self):
        """
        動作準備を行う
        """
        self.m_bcapclient.robot_execute(self.HRobot, "ManualResetPreparation", "")
        self.m_bcapclient.robot_execute(self.HRobot, "Motionpreparation", "")
        print("Motion preapared")

    def get_error_information(self):
        """
        エラー情報を取得する
        """
        return self.m_bcapclient.controller_execute(self.hCtrl,"GetCurErrorInfo",0)[0]

    def get_eroor_count(self):
        """
        発生しているエラーの数を取得する
        """
        return self.m_bcapclient.controller_execute(self.hCtrl,"GetCurErrorCount",0)

    def get_cobotta_status(self):
        """
        COBOTTAがドアの開閉を行っても良い状態かどうかを取得する
        """
        pos=self.get_now_position()
        if 280<pos[0]<300 and -50<pos[1]<-40 and 10<pos[2]<20 and -100<pos[3]<-80 and -10<pos[4]<10 and -100<pos[5]<-80:
            return "Sleep"
        else:
            return "Active"

    def turn_off_motor(self):
        """
        モーターをオフにする
        """
        Command = "Motor"
        Param = [0, 0]
        self.m_bcapclient.robot_execute(self.HRobot, Command, Param)
        print("Motor off")

    ### IO functions ###
    def read_IO_value(self,IO_No):
        """
        指定したIOの値を取得する

        Parameters
        ----------
        IO_No : int
            IO番号

        Returns
        -------
        bool
            IOの値
        """
        IOHandl = 0
        IOHandl = self.m_bcapclient.controller_getvariable(self.hCtrl, f"IO{IO_No}", "")
        # read value
        retIO = self.m_bcapclient.variable_getvalue(IOHandl)
        # Disconnect
        if(IOHandl != 0):
            self.m_bcapclient.variable_release(IOHandl)
            # print(f"Release IO{IO_No}")
        self.m_bcapclient.service_stop()
        return retIO

    def write_IO_value(self,IO_No,IO_Value):
        """
        指定したIOに値を書き込む

        Parameters
        ----------
        IO_No : int
            IO番号
        IO_Value : bool
            書き込む値
        """
        IOHandl = 0
        IOHandl = self.m_bcapclient.controller_getvariable(self.hCtrl, f"IO{IO_No}", "")

        # write value
        self.m_bcapclient.variable_putvalue(IOHandl, IO_Value)
        # read value
        retIO = self.m_bcapclient.variable_getvalue(IOHandl)
        # Disconnect
        if(IOHandl != 0):
            self.m_bcapclient.variable_release(IOHandl)
        self.m_bcapclient.service_stop()
        return retIO


    ### hand functions ###
    def open_hand(self):
        """
        ハンドを開く
        """
        param=[29.91,50]
        self.m_bcapclient.controller_execute(self.hCtrl,"HandMoveA",param)
        print("Hand open")

    def close_hand(self,force=10):
        """
        ハンドを閉じる

        Parameters
        ----------
        force : int, optional
            閉じる力, by default 6
        """
        param=[force,True,"DetectOn"]
        self.m_bcapclient.controller_execute(self.hCtrl,"HandMoveH",param)
        print("Hand close")

    def adjust_hand(self):
        """
        ハンドを調整する
        """
        now_pos= self.m_bcapclient.robot_execute(self.HRobot, "CurJnt")
        if now_pos[5]>-45:
            CobottaController.move_joint6(self,-90)
        else:
            CobottaController.move_joint6(self,90)

    def is_grabbing_sample(self):
        """
        サンプルを掴んでいるかどうかを判定する

        Returns
        -------
        bool
            掴んでいるかどうか
        """
        self.close_hand()
        pos = self.m_bcapclient.controller_execute(self.hCtrl,"HandCurPos")
        if pos > 13.0:
            return True
        else:
            return False


    ### move functions ###
    def go_to_position_parameter(self,param):
        """
        指定した位置に移動する(パス動作)

        Parameters
        ----------
        param : list
            移動先の位置(P型) [x,y,z,rx,ry,rz,fig]
        """
        Pose = [param,"P","@E"]
        self.m_bcapclient.robot_move(self.HRobot,self.COMP,Pose,"")

    def go_to_position_parameter_at_P(self,param):
        """
        指定した位置に移動する(エンコーダ値確認動作)

        Parameters
        ----------
        param : list
            移動先の位置(P型) [x,y,z,rx,ry,rz,fig]
        """
        Pose = [param,"P","@P"]
        self.m_bcapclient.robot_move(self.HRobot,self.COMP,Pose,"")

    def go_to_joint_parameter(self,param):
        """
        指定した関節角に移動する

        Parameters
        ----------
        param : list
            移動先の関節角(J型) [j1,j2,j3,j4,j5,j6]
        """
        Pose = [param,"J","@E"]
        self.m_bcapclient.robot_move(self.HRobot,self.COMP,Pose,"")

    def go_to_position_variable(self,P_No):
        """
        指定した位置に移動する(登録した変数)

        Parameters
        ----------
        P_No : int
            変数番号
        """
        Pose = "P"+str(P_No)
        self.m_bcapclient.robot_move(self.HRobot,self.COMP,Pose,"")

    def go_to_joint_variable(self,J_No):
        """
        指定した関節角に移動する(登録した変数)

        Parameters
        ----------
        J_No : int
            変数番号
        """
        Joint = "J"+str(J_No)
        self.m_bcapclient.robot_move(self.HRobot,self.COMP,Joint,"")

    def shift_parallel(self,x=0.0,y=0.0,z=0.0,rx=0.0,ry=0.0,rz=0.0):
        """
        指定した距離だけ相対的に移動する

        Parameters
        ----------
        x : float, optional
            x方向の移動距離, by default 0
        y : float, optional
            y方向の移動距離, by default 0
        z : float, optional
            z方向の移動距離, by default 0
        rx : float, optional
            x軸周りの回転角度, by default 0
        ry : float, optional
            y軸周りの回転角度, by default 0
        rz : float, optional
            z軸周りの回転角度, by default 0
        """
        now_pos= self.m_bcapclient.robot_execute(self.HRobot, "CurPos")
        target_pos=[now_pos[0]+x,now_pos[1]+y,now_pos[2]+z,now_pos[3]+rx,now_pos[4]+ry,now_pos[5]+rz]
        Pose = [target_pos,"P","@E"]
        self.m_bcapclient.robot_move(self.HRobot,self.COMP,Pose,"")

    def move_joint1(self,param):
        """
        joint1を相対的に回転させる

        Parameters
        ----------
        param : float
            回転角度
        """
        joint_pos=[param]
        param=[joint_pos,"J","@E"]
        self.m_bcapclient.robot_execute(self.HRobot, "DriveEx",param)

    def move_joint6(self,param):
        """
        joint6を相対的に回転させる

        Parameters
        ----------
        param : float
            回転角度
        """
        joint_pos=[0,0,0,0,0,param]
        param=[joint_pos,"J","@E"]
        self.m_bcapclient.robot_execute(self.HRobot, "DriveEx",param)

    def go_to_sleep_position(self):
        """
        ホームポジション(サンプルホテル横)に移動する
        """
        self.go_to_position_parameter(self.fixed_positions["sleep"])
        self.open_hand()
        self.turn_off_motor()

    def move_in_a_spiral(self,center,radius_max,rotation_number):
        """
        指定した中心を中心とした螺旋状に移動する

        Parameters
        ----------
        center : list
            中心の位置(P型) [x,y,z,rx,ry,rz,fig]
        radius_max : float
            最大半径(mm)
        rotation_number : int
            回転数
        """
        for i in range(0,360*rotation_number,10):
            x = center[0] + radius_max/(rotation_number*2*math.pi)*math.radians(i)*math.cos(math.radians(i))
            y = center[1] + radius_max/(rotation_number*2*math.pi)*math.radians(i)*math.sin(math.radians(i))
            self.go_to_position_parameter_at_P([x,y,center[2]-6,center[3],center[4],center[5],center[6]])
        self.shift_parallel(z=5)


    ### door functions ###
    def open_door(self):
        """
        ドアを開ける
        """
        if self.get_door_status()=="Close":
            if self.get_cobotta_status()=="Sleep":
                self.write_IO_value(25,False)
                self.write_IO_value(26,True)
                while self.read_IO_value(9)==False:
                    time.sleep(0.1)
                else:
                    time.sleep(0.3)
                    self.write_IO_value(25,True)
                    self.write_IO_value(26,True)
                    print("Door open")
                if self.get_error_information()==-2075524850:
                    self.clear_error()
                    self.motion_preparation()
        elif self.get_door_status()=="Open":
            pass
        else:
            print("Error: Failed to open door")
            exit()

    def close_door(self):
        """
        ドアを閉じる
        """
        if self.get_door_status()=="Open":
            if self.get_cobotta_status()=="Sleep":
                self.write_IO_value(25,True)
                self.write_IO_value(26,False)
                while self.read_IO_value(8)==False:
                    time.sleep(0.1)
                else:
                    time.sleep(0.1)
                    self.write_IO_value(25,True)
                    self.write_IO_value(26,True)
                    print("Door close")
                if self.get_error_information()==-2075524850:
                    self.clear_error()
                    self.motion_preparation()
        elif self.get_door_status()=="Close":
            pass
        else:
            print("Error: Failed to close door")
            exit()

    def get_door_status(self):
        """
        ドアの開閉状態を取得する

        Returns
        -------
        str

            "Open" : 開いている
            "Close" : 閉じている
            "Neutral" : 中間状態
            "Error" : エラー
        """
        door_status=(self.read_IO_value(8),self.read_IO_value(9))
        if door_status==(False,True):
            return "Open"
        elif door_status==(True,False):
            return "Close"
        elif door_status==(False,False):
            return "Neutral"
        elif door_status==(True,True):
            return "Error"


    ### tower functions ### floor=0~11
    def open_tower(self,floor):
        """
        サンプルホテルを開く

        Parameters
        ----------
        floor : int
            開く階数(0~11)
        """
        self.change_speed(100,100,100)
        home = [343.36,28.08,18.3+self.FLOOR_HEIGHT*floor,-87.43,89.41,-80.85,5.0]
        self.go_to_position_parameter(home)
        self.move_joint1(-7.5)
        self.shift_parallel(z=4)
        self.change_speed(20,100,100)
        self.move_joint1(14.6)
        time.sleep(0.5)
        self.move_joint1(-0.5)
        self.change_speed(100,100,100)
        self.shift_parallel(0,0,-5,0,0,0)
        self.move_joint1(5)
        self.shift_parallel(0,0,10,0,0,0)
        self.shift_parallel(-70,0,0,0,0,0)
        self.adjust_hand()

    def close_tower(self,floor):
        """
        サンプルホテルを閉じる

        Parameters
        ----------
        floor : int
            閉じる階数(0~11)
        """
        self.change_speed(100,100,100)
        home = [263.04,88.39,29.40+self.FLOOR_HEIGHT*floor,-89.95,-0.60,-73.22,5.0]
        self.go_to_position_parameter(home)
        self.adjust_hand()
        self.shift_parallel(70,0,0,0,0,0)
        self.change_speed(20,100,100)
        self.move_joint1(-17)
        self.change_speed(100,100,100)
        time.sleep(0.2)
        self.move_joint1(17)
        self.adjust_hand()
        self.go_to_position_parameter(home)


    ### mounting functions ###
    def mount_sample(self,center,radius,rotation_number):
        delta_z = 3/rotation_number/360
        for i in range(0,360*rotation_number,1):
            x = center[0] + radius*math.cos(math.radians(i))
            y = center[1] + radius*math.sin(math.radians(i))
            z = center[2] - delta_z*i
            self.go_to_position_parameter_at_P([x,y,z,center[3],center[4],center[5],center[6]])
        self.shift_parallel(z=1)
        try:
            self.change_speed(speed=10)
            self.shift_parallel(z=-4)
            time.sleep(1)
        except:
            self.turn_off_motor()
            self.clear_error()
            self.motion_preparation()
            self.shift_parallel(z=10)
        self.shift_parallel(z=20)
        self.change_speed()
        self.move_joint1(-10)

    def open_funnel(self):
        """
        漏斗を開く
        """
        self.go_to_position_parameter_at_P(self.fixed_positions["before_funnel_drawer"])
        self.move_joint1(13)
        self.change_speed(speed=20)
        self.shift_parallel(z=-15)
        self.move_joint1(-16)
        time.sleep(0.5)
        self.move_joint1(0.5)
        self.change_speed()
        self.shift_parallel(z=7)
        self.move_joint1(-5)

    def close_funnel(self):
        """
        漏斗を閉じる
        """
        self.go_to_position_parameter_at_P(self.fixed_positions["before_closing_funnel"])
        self.change_speed(speed=20)
        self.move_joint1(19.5)
        time.sleep(0.5)
        self.move_joint1(-1)
        self.change_speed()
        self.go_to_position_parameter_at_P(self.fixed_positions["before_closing_funnel"])


    ### paper functions ###
    def attach_paper(self):
        """
        薬包紙を取り付ける
        """
        self.go_to_joint_parameter([49.39, 77.81, 97.8, -0.61, -85.33, -90])
        self.change_speed(20)
        self.shift_parallel(z=-18)
        time.sleep(0.5)
        self.shift_parallel(z=20)
        self.change_speed()

    def detach_paper(self):
        """
        薬包紙を取り外す
        """
        self.go_to_joint_parameter([-21.73, 35.9, 121.31, -1.76, -67.2, -89.46])
        self.change_speed(15)
        self.shift_parallel(z=-20)
        time.sleep(0.5)
        self.shift_parallel(z=25)
        self.change_speed()
        self.move_joint1(23.58)

    ### sample transport functions ###
    def transport_from_tower_to_platform(self,floor,room,funnel=True):
        """
        サンプルをサンプルホテルから作製台に移動する
        
        Parameters
        ----------
        floor : int
            移動元の階数(0~11)
        room : int
            移動元の部屋番号(0~1)
        funnel : bool, optional
            漏斗を使用するかどうか, by default True
        """
        pos_in_front_of_tower = [310.11,5.62,35.55+self.FLOOR_HEIGHT*floor,-90,-1.6,-90,5.0]
        self.open_tower(floor)
        self.move_joint1(-17)
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.shift_parallel(y=-self.TOWER_DELTA_Y*room)
        self.shift_parallel(x=51)
        self.shift_parallel(z=3)
        self.close_hand()
        self.shift_parallel(z=1.5)
        time.sleep(0.5)
        self.open_hand()
        self.shift_parallel(x=-51)
        self.shift_parallel(y=self.TOWER_DELTA_Y*room)
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.move_joint1(17)
        self.go_to_position_parameter(self.fixed_positions["in_front_of_platform"])
        self.go_to_position_parameter(self.fixed_positions["above_platform"])
        self.change_speed(speed=10)
        self.shift_parallel(z=-7)
        time.sleep(0.5)
        self.shift_parallel(z=7)
        self.change_speed(speed=100)
        self.go_to_position_parameter(self.fixed_positions["in_front_of_platform"])
        self.shift_parallel(z=30)
        if funnel:
            self.shift_parallel(z=30)
            self.open_funnel()
        self.close_tower(floor)
        self.go_to_sleep_position()
        print("Sample set to Platform")

    def transport_from_platform_to_miniflex(self,mount=True,funnel=True):
        """
        サンプルを作製台からMiniFlexに移動する

        Parameters
        ----------
        mount : bool, optional
            サンプルをマウントするかどうか, by default True
        funnel : bool, optional
            漏斗を使用するかどうか, by default True
        """
        self.open_door()
        if mount:
            self.attach_paper()
            if funnel:
                self.close_funnel()
            self.go_to_position_parameter(self.fixed_positions["before_mounting"])
            self.go_to_position_parameter(self.fixed_positions["center_of_platform"])
            self.mount_sample(self.fixed_positions["mount_center"],0.5,1)
            self.shift_parallel(y=-40)
        self.go_to_position_parameter(self.fixed_positions["in_front_of_platform"])
        self.go_to_position_parameter(self.fixed_positions["above_platform"])
        self.shift_parallel(z=5)
        self.close_hand()
        time.sleep(0.5)
        self.open_hand()
        self.shift_parallel(z=-3)
        self.go_to_position_parameter_at_P(self.fixed_positions["in_front_of_platform"])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        for i in (0,1,2):
            self.go_to_position_parameter_at_P(self.fixed_positions["in_miniflex"][i])
        self.change_speed(10,50,100)
        for i in (3,4):
            self.go_to_position_parameter(self.fixed_positions["in_miniflex"][i])
        time.sleep(1)
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][3])
        self.change_speed(100,100,100)
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][2])
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][1])
        if mount:
            self.go_to_joint_parameter([-21.2, 0.91, 95.82, 2.18, -8.72, -88.84])
            self.detach_paper()
        else:
            self.go_to_position_parameter(self.fixed_positions["in_miniflex"][0])
            self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
            self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.go_to_position_parameter_at_P(self.fixed_positions["sleep"])
        time.sleep(0.5)
        self.close_door()
        print("Sample set to MiniFlex")

    def transport_from_miniflex_to_tower(self,floor,room):
        """
        サンプルをMiniFlexからサンプルホテルに移動する

        Parameters
        ----------
        floor : int
            移動先の階数(0~11)
        room : int
            移動先の部屋番号(0~1)
        """
        self.open_door()
        self.open_tower(floor)
        self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        for i in (0,1,2):
            self.go_to_position_parameter_at_P(self.fixed_positions["in_miniflex"][i])
        self.change_speed(10,50,100)
        for i in (3,5):
            self.go_to_position_parameter(self.fixed_positions["in_miniflex"][i])
        self.close_hand()
        time.sleep(0.5)
        self.open_hand()
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][3])
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][2])
        self.change_speed(100,100,100)
        for i in (1,0):
            self.go_to_position_parameter_at_P(self.fixed_positions["in_miniflex"][i])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.go_to_position_parameter([294.92,96.03,32.55+self.FLOOR_HEIGHT*floor,-90,0,-73,5.0])
        waiting_position = [310.11,5.62,32.55+self.FLOOR_HEIGHT*floor,-90,0,-90,5.0]
        self.go_to_position_parameter(waiting_position)
        self.shift_parallel(x=51,y=-self.TOWER_DELTA_Y*room)
        self.change_speed(10,100,100)
        self.shift_parallel(z=-5)
        time.sleep(0.5)
        self.shift_parallel(z=5)
        self.change_speed(100,100,100)
        self.shift_parallel(x=-51,y=self.TOWER_DELTA_Y*room)
        self.go_to_position_parameter(waiting_position)
        self.move_joint1(17)
        self.close_tower(floor)
        self.go_to_position_parameter_at_P(self.fixed_positions["sleep"])
        time.sleep(0.5)
        self.close_door()
        print(f"Sample set to Tower {floor}-{room}")

    def transport_from_tower_to_miniflex(self,floor,room):
        """
        サンプルをサンプルホテルからMiniFlexに移動する

        Parameters
        ----------
        floor : int
            移動元の階数(0~11)
        room : int
            移動元の部屋番号(0~1)
        """
        self.open_door()
        pos_in_front_of_tower = [310.11,5.62,35.55+self.FLOOR_HEIGHT*floor,-90,-1.6,-90,5.0]
        self.open_tower(floor)
        self.move_joint1(-17)
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.shift_parallel(y=-self.TOWER_DELTA_Y*room)
        self.shift_parallel(x=51)
        self.shift_parallel(z=3)
        self.close_hand()
        self.shift_parallel(z=1.5)
        time.sleep(0.5)
        self.open_hand()
        self.shift_parallel(x=-51)
        self.shift_parallel(y=self.TOWER_DELTA_Y*room)
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.move_joint1(17)
        self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        for i in (0,1,2):
            self.go_to_position_parameter_at_P(self.fixed_positions["in_miniflex"][i])
        self.change_speed(10,50,100)
        for i in (3,4):
            self.go_to_position_parameter(self.fixed_positions["in_miniflex"][i])
        time.sleep(1)
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][3])
        self.change_speed(100,100,100)
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][2])
        for i in (1,0):
            self.go_to_position_parameter_at_P(self.fixed_positions["in_miniflex"][i])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.close_tower(floor)
        self.go_to_position_parameter_at_P(self.fixed_positions["sleep"])
        time.sleep(0.5)
        self.close_door()
        print("Sample Set to MiniFlex")


    def transport_from_tower_to_platform_2(self,floor,room,funnel=True):
        """
        サンプルをサンプルホテルから作製台に移動する
        
        Parameters
        ----------
        floor : int
            移動元の階数(0~11)
        room : int
            移動元の部屋番号(0~1)
        funnel : bool, optional
            漏斗を使用するかどうか, by default True
        """
        pos_in_front_of_tower = [310.11,5.62-self.TOWER_DELTA_Y*room,35.55+self.FLOOR_HEIGHT*floor,-90,-1.6,-90,5.0]
        target_room_position = self.variable_positions["hotel_0-0"].copy()
        target_room_position[1] -= self.TOWER_DELTA_Y*room
        target_room_position[2] += self.FLOOR_HEIGHT*floor

        self.open_tower(floor)
        self.move_joint1(-17)
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.go_to_position_parameter(target_room_position)
        self.shift_parallel(z=5)
        self.close_hand()
        time.sleep(1)
        self.open_hand()
        self.shift_parallel(z=-2)
        self.shift_parallel(x=-51)
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.move_joint1(17)
        self.go_to_position_parameter(self.fixed_positions["in_front_of_platform"])
        self.go_to_position_parameter(self.variable_positions["platform_stage"])
        self.change_speed(speed=10)
        self.shift_parallel(z=-5)
        time.sleep(1)
        self.shift_parallel(z=5)
        self.change_speed(speed=100)
        self.go_to_position_parameter(self.fixed_positions["in_front_of_platform"])
        if funnel:
            self.shift_parallel(z=50)
            self.open_funnel()
        self.close_tower(floor)
        self.go_to_sleep_position()
        print("Sample set to Platform")
        

    def transport_from_platform_to_miniflex_2(self,mount=True,funnel=True):
        """
        サンプルを作製台からMiniFlexに移動する

        Parameters
        ----------
        mount : bool, optional
            サンプルをマウントするかどうか, by default True
        funnel : bool, optional
            漏斗を使用するかどうか, by default True
        """
        self.open_door()
        if mount:
            self.attach_paper()
            if funnel:
                self.close_funnel()
            self.go_to_position_parameter(self.fixed_positions["before_mounting"])
            self.go_to_position_parameter(self.fixed_positions["center_of_platform"])
            self.mount_sample(self.fixed_positions["mount_center"],0.5,1)
            self.shift_parallel(y=-40)
        self.go_to_position_parameter(self.fixed_positions["in_front_of_platform"])
        self.go_to_position_parameter(self.variable_positions["platform_stage"])
        self.shift_parallel(z=3.5)
        self.close_hand()
        time.sleep(1)
        self.open_hand()
        self.shift_parallel(z=-3)
        self.go_to_position_parameter_at_P(self.fixed_positions["in_front_of_platform"])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        for i in (0,1,2):
            self.go_to_position_parameter_at_P(self.fixed_positions["in_miniflex"][i])
        self.change_speed(10,50,100)
        self.go_to_position_parameter(self.variable_positions["miniflex_5"])
        self.shift_parallel(z=-5)
        time.sleep(1)
        self.go_to_position_parameter(self.variable_positions["miniflex_5"])
        self.change_speed(100,100,100)
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][2])
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][1])
        if mount:
            self.go_to_joint_parameter([-21.2, 0.91, 95.82, 2.18, -8.72, -88.84])
            self.detach_paper()
        else:
            self.go_to_position_parameter(self.fixed_positions["in_miniflex"][0])
            self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
            self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.go_to_position_parameter_at_P(self.fixed_positions["sleep"])
        time.sleep(0.5)
        self.close_door()
        print("Sample set to MiniFlex")


    def transport_from_miniflex_to_tower_2(self,floor,room):
        """
        サンプルをMiniFlexからサンプルホテルに移動する

        Parameters
        ----------
        floor : int
            移動先の階数(0~11)
        room : int
            移動先の部屋番号(0~1)
        """
        pos_in_front_of_tower = [310.11,5.62-self.TOWER_DELTA_Y*room,35.55+self.FLOOR_HEIGHT*floor,-90,-1.6,-90,5.0]
        target_room_position = self.variable_positions["hotel_0-0"].copy()
        target_room_position[1] -= self.TOWER_DELTA_Y*room
        target_room_position[2] += self.FLOOR_HEIGHT*floor

        self.open_door()
        self.open_tower(floor)
        self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        for i in (0,1,2):
            self.go_to_position_parameter_at_P(self.fixed_positions["in_miniflex"][i])
        self.change_speed(10,50,100)
        self.go_to_position_parameter(self.variable_positions["miniflex_5"])
        self.shift_parallel(z=4.5)
        self.close_hand()
        time.sleep(1)
        self.open_hand()
        self.go_to_position_parameter(self.variable_positions["miniflex_5"])
        self.change_speed(100,100,100)
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][2])
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][1])
        self.go_to_position_parameter(self.fixed_positions["in_miniflex"][0])
        self.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
        self.go_to_position_parameter(self.fixed_positions["before_miniflex"])
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.go_to_position_parameter(target_room_position)
        self.change_speed(speed=10)
        self.shift_parallel(z=-5)
        time.sleep(1)
        self.shift_parallel(z=5)
        self.change_speed(speed=100)
        self.shift_parallel(x=-51)
        self.go_to_position_parameter(pos_in_front_of_tower)
        self.move_joint1(17)
        self.close_tower(floor)
        self.go_to_sleep_position()
        time.sleep(0.5)
        self.close_door()
        print("Sample set to Tower {floor}-{room}")



if __name__ == '__main__':
    cc = CobottaController()
    # cc.connect()
    # cc.go_to_sleep_position()
    cc.auto_calibration()
    # cc.disconnect()