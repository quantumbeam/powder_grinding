#!/usr/bin/env python3


from inspect import stack
import rospy
from rospy.exceptions import ROSException
from rospy.timer import sleep

from grinding_vision.srv import DiagonalPowderRadius
from kek_sound.srv import GatheringFlagWithSound

import time
import datetime
import copy
from inputimeout import inputimeout, TimeoutOccurred
import pandas as pd


import motion_routines
from grinding_primitive import *


################### Fixed params ###################

# experiments params
TIMEOUT_SEC = 0.001


class VisionSoundGrinding:
    def __init__(self, vision_server, sound_server) -> None:
        # grinding params
        self.pestle_radius = rospy.get_param("~pestle_radius")
        self.minimum_grinding_radius_mm = rospy.get_param("~minimum_grinding_radius")
        self.grinding_eef_link = rospy.get_param("~grinding_eef_link")
        self.gathering_eef_link = rospy.get_param("~gathering_eef_link")
        self.grinding_yaw_angle = rospy.get_param("~grinding_yaw_angle")
        self.last_motion = "gathering"

        # vision variables
        self.vision_server = vision_server
        self.grinding_radius_threshold = rospy.get_param("~vision_threshold")
        self.last_vision_radius = 0
        if rospy.get_param("~arm_moving_at_vision") == False:
            vision_server(stack_sec=0, start_stack_frame=True)  # init start stack frame
            time.sleep(1)

        # sound variables
        self.sound_server = sound_server
        self.sound_RMS_threshold = rospy.get_param("~sound_RMS_threshold")
        self.sound_RMS = 0

        # debug variables
        self.vision_radius_list = []
        self.grinding_radius_list = []
        self.stack_sec_list = []
        self.sound_RMS_list = []
        self.motion_list = []
        self.now = datetime.datetime.now()

    def update_grinding_radius(self, vision_radius):
        grinding_radius = vision_radius - self.pestle_radius
        if grinding_radius < self.minimum_grinding_radius_mm:
            grinding_radius = self.minimum_grinding_radius_mm
        return grinding_radius

    def export_debug(self, experimental_time_list):
        debug_df = pd.DataFrame(
            list(
                zip(
                    experimental_time_list,
                    self.vision_radius_list,
                    self.grinding_radius_list,
                    self.stack_sec_list,
                    self.sound_RMS_list,
                    self.motion_list,
                )
            ),
            columns=(
                "experimental_time",
                "vision_radius",
                "grinding_radius",
                "stack_sec",
                "sound_RMS",
                "motion_list",
            ),
        )

        filepath = (
            roslib.packages.get_pkg_dir("grinding_motion_routines")
            + "/config/"
            + self.now.strftime("%Y%m%d_%H%M%S")
            + "_debug.csv"
        )
        debug_df.to_csv(
            filepath,
            header=True,
            index=False,
        )

    def vison_grinding(
        self,
        motion_routines_class,
        grinding_eef_link,
        gathering_eef_link,
    ):
        # get powder radius
        vision_result = self.vision_server(stack_sec=0, start_stack_frame=False)
        vision_radius = vision_result.radius
        if vision_radius < 0:
            exit_demo("error: 0 > vision radius detected")

        # grinding radius
        grinding_radius = self.update_grinding_radius(vision_radius)
        rospy.loginfo("Vision radius as " + str(vision_radius))

        if vision_radius > self.last_vision_radius:
            # rospy.loginfo("Decide grinding with radius as " + str(grinding_radius))

            # goto ready pose
            if self.last_motion == "gathering":
                goto_grinding_ready_pose(
                    motion_routines_class,
                    self.grinding_eef_link,
                    vel_scale=0.5,
                    acc_scale=0.5,
                )
            if rospy.get_param("~fixed_grinding"):
                begin_pos = rospy.get_param("~grinding_pos_begining")
                end_pos = rospy.get_param("~grinding_pos_end")
                grinding_radius = abs(begin_pos[0])
            else:
                begin_pos = [-grinding_radius, 0]
                end_pos = [-grinding_radius, 0.0001]
            rospy.loginfo("Grinding radius as " + str(grinding_radius))
            (
                planning_success,
                waypoints,
                plan,
                grinding_sec,
            ) = get_plan_and_time_for_grinding(
                motion_routines_class,
                begin_pos,
                end_pos,
                grinding_eef_link,
            )

            # set grinding time as stack_sec for vision
            grinding_sec_once = grinding_sec / rospy.get_param(
                "~grinding_number_of_rotation"
            )
            self.vision_server(stack_sec=grinding_sec_once, start_stack_frame=False)
            self.stack_sec = grinding_sec_once

            # execute
            if planning_success:
                motion_routines_class.move_group.execute(plan)

            sound_result = 1  # self.sound_server(grinding_sec=grinding_sec_once * 2)
            self.sound_RMS = sound_result.sound_RMS

            self.last_vision_radius = vision_radius
            # self.last_vision_radius = 100
            self.last_motion = "grinding"

        else:
            # rospy.loginfo("Decide gathering")
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            planning_success = plan_and_execute_circular_gathering(
                motion_routines_class,
                gathering_eef_link,
                execute=True,
            )
            goto_init_pose(
                motion_routines_class,
                self.gathering_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )

            # set start stack frame
            grinding_sec_once = 0
            self.vision_server(stack_sec=grinding_sec_once, start_stack_frame=True)

            # time.sleep(self.stack_sec)  # wait to         print(self.img_list[0].shape)

            self.last_vision_radius = 0
        self.vision_radius_list.append(vision_radius)
        self.grinding_radius_list.append(grinding_radius)
        self.stack_sec_list.append(grinding_sec_once)
        self.sound_RMS_list.append(self.sound_RMS)
        self.motion_list.append(self.last_motion)

        if planning_success == False:
            rospy.loginfo("Get out lock condition")
            get_out_lock_condition(motion_routines_class)
            planning_success = True

    def IROS_vison_grinding(
        self,
        motion_routines_class,
        grinding_eef_link,
        gathering_eef_link,
    ):
        # get powder radius
        if rospy.get_param("~arm_moving_at_vision"):
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=1,
                acc_scale=1,
            )
        vision_result = self.vision_server(stack_sec=0, start_stack_frame=False)
        vision_radius = vision_result.radius
        if vision_radius < 0:
            exit_demo("error: 0 > vision radius detected")

        # grinding radius
        grinding_radius = self.update_grinding_radius(vision_radius)
        rospy.loginfo("Vision radius as " + str(vision_radius))

        # grinding and gathering decision
        if rospy.get_param("~fixed_vision_threshold"):
            if (
                vision_radius < self.grinding_radius_threshold
                or self.last_motion == "gathering"
            ):
                grinding_flag = True
            else:
                grinding_flag = False
        else:
            if vision_radius > self.last_vision_radius:
                grinding_flag = True
            else:
                grinding_flag = False

        if grinding_flag:
            # rospy.loginfo("Decide grinding with radius as " + str(grinding_radius))

            # goto ready pose
            if self.last_motion == "gathering":
                goto_grinding_ready_pose(
                    motion_routines_class,
                    self.grinding_eef_link,
                    vel_scale=0.5,
                    acc_scale=0.5,
                )
            if rospy.get_param("~fixed_grinding"):
                begin_pos = rospy.get_param("~grinding_pos_begining")
                end_pos = rospy.get_param("~grinding_pos_end")
                grinding_radius = abs(begin_pos[0])
            else:
                begin_pos = [-grinding_radius, 0]
                end_pos = [-grinding_radius, 0.0001]
            rospy.loginfo("Grinding radius as " + str(grinding_radius))
            (
                planning_success,
                waypoints,
                plan,
                grinding_sec,
            ) = get_plan_and_time_for_grinding(
                motion_routines_class,
                begin_pos,
                end_pos,
                grinding_eef_link,
            )

            # set grinding time as stack_sec for vision
            grinding_sec_once = grinding_sec / rospy.get_param(
                "~grinding_number_of_rotation"
            )
            if rospy.get_param("~arm_moving_at_vision") == False:
                self.vision_server(stack_sec=grinding_sec_once, start_stack_frame=False)
                self.stack_sec = grinding_sec_once

            # execute
            if planning_success:
                motion_routines_class.move_group.execute(plan)

            sound_result = self.sound_server(grinding_sec=grinding_sec_once * 2)
            self.sound_RMS = sound_result.sound_RMS

            self.last_vision_radius = vision_radius
            self.last_motion = "grinding"

        else:
            # rospy.loginfo("Decide gathering")
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            planning_success = plan_and_execute_circular_gathering(
                motion_routines_class,
                gathering_eef_link,
                execute=True,
            )
            goto_init_pose(
                motion_routines_class,
                self.gathering_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )

            # set start stack frame
            if rospy.get_param("~arm_moving_at_vision") == False:
                self.vision_server(stack_sec=0, start_stack_frame=True)
                # time.sleep(self.stack_sec)  # wait to get right stack image
            grinding_radius = 0
            self.last_vision_radius = 0
            self.last_motion = "gathering"
        self.vision_radius_list.append(vision_radius)
        self.grinding_radius_list.append(grinding_radius)
        self.stack_sec_list.append(0)
        self.sound_RMS_list.append(self.sound_RMS)
        self.motion_list.append(self.last_motion)

        if planning_success == False:
            rospy.loginfo("Get out lock condition")
            get_out_lock_condition(motion_routines_class)
            planning_success = True

    def vison_sound_grinding(
        self,
        motion_routines_class,
        grinding_eef_link,
        gathering_eef_link,
    ):
        # get powder radius
        vision_result = self.vision_server(stack_sec=0, start_stack_frame=False)
        vision_radius = vision_result.radius
        if vision_radius < 0:
            exit_demo("error: 0 > vision radius detected")
        rospy.loginfo("Vision radius as " + str(vision_radius))

        # decide next motion
        if self.sound_RMS > self.sound_RMS_threshold:
            self.next_motion = "grinding"
        else:
            if vision_radius > self.last_vision_radius:
                # if vision_radius < self.grinding_radius_threshold:
                self.grinding_radius = self.update_grinding_radius(vision_radius)
                self.next_motion = "grinding"
            else:
                self.grinding_radius = 0
                self.next_motion = "gathering"

            self.last_vision_radius = vision_radius

        if self.next_motion == "grinding":
            rospy.loginfo("Grinding radius as " + str(self.grinding_radius))

            # goto ready pose
            if self.last_motion == "gathering":
                goto_grinding_ready_pose(
                    motion_routines_class,
                    self.grinding_eef_link,
                    vel_scale=0.5,
                    acc_scale=0.5,
                )

            begin_pos = [-self.grinding_radius, 0]
            end_pos = [-self.grinding_radius, 0.0001]
            (
                planning_success,
                waypoints,
                plan,
                grinding_sec,
            ) = get_plan_and_time_for_grinding(
                motion_routines_class,
                begin_pos,
                end_pos,
                grinding_eef_link,
            )

            # set grinding time as stack_sec for vision
            grinding_sec_once = grinding_sec / rospy.get_param(
                "~grinding_number_of_rotation"
            )
            self.vision_server(stack_sec=grinding_sec_once, start_stack_frame=False)
            self.stack_sec = grinding_sec_once

            # set grinding time as threshold sec for sound
            self.sound_server(grinding_sec=grinding_sec_once * 2)

            # execute
            if planning_success:
                motion_routines_class.move_group.execute(plan)

            # get sound
            sound_result = self.sound_server(grinding_sec=grinding_sec_once * 2)
            self.sound_RMS = sound_result.sound_RMS
            rospy.loginfo("Sound RMS as " + str(self.sound_RMS))

            self.last_motion = "grinding"

        elif self.next_motion == "gathering":
            # rospy.loginfo("Decide gathering")
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            planning_success = plan_and_execute_circular_gathering(
                motion_routines_class,
                gathering_eef_link,
                execute=True,
            )
            goto_init_pose(
                motion_routines_class,
                self.gathering_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )
            goto_init_pose(
                motion_routines_class,
                self.grinding_eef_link,
                vel_scale=0.5,
                acc_scale=0.5,
            )

            # init threshold values
            self.sound_RMS = 0
            self.last_vision_radius = 0

            # set start stack frame
            grinding_sec_once = 0
            self.vision_server(stack_sec=grinding_sec_once, start_stack_frame=True)

            # time.sleep(self.stack_sec)  # wait to get right stack image
            self.last_motion = "gathering"
            self.next_motion == "grinding"

        # debug
        self.vision_radius_list.append(vision_radius)
        self.grinding_radius_list.append(self.grinding_radius)
        self.stack_sec_list.append(grinding_sec_once)
        self.sound_RMS_list.append(self.sound_RMS)
        self.motion_list.append(self.last_motion)

        if planning_success == False:
            rospy.loginfo("Get out lock condition")
            get_out_lock_condition(motion_routines_class)
            planning_success = True


def main():
    global last_motion

    ################### init node ###################
    rospy.init_node("move_group_python_interface", anonymous=True)

    ################### read params ###################
    experimental_time = rospy.get_param("~experimental_time")
    move_group_name = rospy.get_param("~move_group_name")
    grinding_eef_link = rospy.get_param("~grinding_eef_link")
    gathering_eef_link = rospy.get_param("~gathering_eef_link")
    motion_routines_class = motion_routines.MotionRoutine(
        move_group_name, grinding_eef_link
    )

    rospy.loginfo("Wait for diagonal_vision service")
    try:
        rospy.wait_for_service("/diagonal_vision", timeout=5.0)
        vision_server = rospy.ServiceProxy(
            "/diagonal_vision", DiagonalPowderRadius, persistent=True
        )
    except ROSException:
        rospy.logerr("timeout grinding_vision service")
        exit()

    # rospy.loginfo("Wait for sound service")
    # try:
    #     rospy.wait_for_service("/kek_sound", timeout=5.0)
    #     sound_server = rospy.ServiceProxy(
    #         "/kek_sound", GatheringFlagWithSound, persistent=True
    #     )
    # except ROSException:
    #     rospy.logerr("timeout kek_sound service")
    #     exit()
    sound_server = None
    ################### init planning scene ###################
    init_planning_scene(motion_routines_class)

    ################### init pose ###################
    goto_init_pose(motion_routines_class, grinding_eef_link)

    motion_result = True

    try:
        while not rospy.is_shutdown():
            motion_command = input(
                "Input command.\n\n q \t\t\t= exit.\n"
                + "scene \t\t\t= reload planning scene.\n "
                + "grinding, gathering \t= g -> grinding,G -> gathering.\n"
                + "calib_hight \t\t= go to calibration pose of mortar hight.\n"
                + "continue grinding \t\t\t= only grinding for "
                + str(experimental_time)
                + " [min] .\n"
                + "continue GG \t\t\t= grinding and gathering for "
                + str(experimental_time)
                + " [min] .\n"
                + "VF \t\t\t= grinding with visual feedback for "
                + str(experimental_time)
                + " [min] .\n"
                + "VSF \t\t\t= grinding with visual and sound feedback for "
                + str(experimental_time)
                + " [min] .\n"
                + "\n"
            )

            if motion_command == "q":
                exit_demo()

            elif motion_command == "scene":
                rospy.loginfo("Init planning scene")
                init_planning_scene(motion_routines_class)

            elif motion_command == "g":
                key = input(
                    "Start grinding motion.\n execute = 'y', step by step = 's', continue grinding = 'c', canncel = other\n"
                )
                if command_to_execute(key) != None:
                    motion_result = plan_and_execute_grinding(
                        motion_routines_class,
                        rospy.get_param("~grinding_pos_begining"),
                        rospy.get_param("~grinding_pos_end"),
                        grinding_eef_link,
                        execute=command_to_execute(key),
                    )
            elif motion_command == "G":
                key = input(
                    "Start gathering motion.\n execute = 'y', step by step = 's',  canncel = other\n"
                )
                if command_to_execute(key) != None:
                    motion_result = plan_and_execute_circular_gathering(
                        motion_routines_class,
                        gathering_eef_link,
                        execute=command_to_execute(key),
                    )

            elif motion_command == "GG":
                while True:
                    try:
                        key = inputimeout(
                            prompt="If you want to finish grinding, Prease enter 'q'.\n",
                            timeout=TIMEOUT_SEC,
                        )
                        if key == "q":
                            exit_demo()
                    except TimeoutOccurred:
                        # init vision stack frame
                        _ = VisionSoundGrinding(vision_server, sound_server)
                        motion_result = plan_and_execute_grinding(
                            motion_routines_class,
                            rospy.get_param("~grinding_pos_begining"),
                            rospy.get_param("~grinding_pos_end"),
                            grinding_eef_link,
                            execute=command_to_execute("y"),
                        )
                        motion_result = plan_and_execute_circular_gathering(
                            motion_routines_class,
                            gathering_eef_link,
                            execute=command_to_execute("y"),
                        )

            elif (
                motion_command == "continue grinding"
                or motion_command == "continue GG"
                or motion_command == "VF"
                or motion_command == "IROS_VF"
                or motion_command == "VSF"
            ):
                motion_counts = 0
                experimenting_time_list = []
                st = time.time()
                # inti vision sound grinding class
                vision_grinding = VisionSoundGrinding(vision_server, sound_server)
                while True:
                    try:
                        key = inputimeout(
                            prompt="If you want to finish grinding, Prease enter 'q'.\n",
                            timeout=TIMEOUT_SEC,
                        )
                        if key == "q":
                            exit_demo()

                    except TimeoutOccurred:
                        if motion_command == "continue grinding":
                            motion_result = plan_and_execute_grinding(
                                motion_routines_class,
                                rospy.get_param("~grinding_pos_begining"),
                                rospy.get_param("~grinding_pos_end"),
                                grinding_eef_link,
                            )
                        elif motion_command == "continue GG":
                            motion_result = plan_and_execute_grinding(
                                motion_routines_class,
                                rospy.get_param("~grinding_pos_begining"),
                                rospy.get_param("~grinding_pos_end"),
                                grinding_eef_link,
                                execute=command_to_execute("y"),
                            )
                            motion_result = plan_and_execute_circular_gathering(
                                motion_routines_class,
                                gathering_eef_link,
                                execute=command_to_execute("y"),
                            )
                        elif motion_command == "VF":
                            vision_grinding.vison_grinding(
                                motion_routines_class,
                                grinding_eef_link,
                                gathering_eef_link,
                            )
                        elif motion_command == "IROS_VF":
                            vision_grinding.IROS_vison_grinding(
                                motion_routines_class,
                                grinding_eef_link,
                                gathering_eef_link,
                            )
                        elif motion_command == "VSF":
                            vision_grinding.vison_sound_grinding(
                                motion_routines_class,
                                grinding_eef_link,
                                gathering_eef_link,
                            )
                        motion_counts += 1

                    experimenting_time = (time.time() - st) / 60
                    rospy.loginfo(
                        "Experiment time: " + str(experimenting_time) + " min"
                    )
                    experimenting_time_list.append(experimenting_time)
                    vision_grinding.export_debug(experimenting_time_list)
                    if experimenting_time > experimental_time:
                        rospy.loginfo("Over experiment time")
                        exit_demo("Motion counts: " + str(motion_counts))

            elif motion_command == "calib_hight":
                mortar_hight_calibration(motion_routines_class)

            if motion_result == False:
                rospy.loginfo("Get out lock condition")
                get_out_lock_condition(motion_routines_class)
                motion_result = True

    except rospy.ROSInterruptException as err:
        exit_demo(err)
    except KeyboardInterrupt as err:
        exit_demo(err)


if __name__ == "__main__":
    main()
