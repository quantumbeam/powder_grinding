import socket
import cobotta_controller as cobotta_controller
import flet as ft
import time


global joint_list
if __name__ == "__main__":
    cobotta = cobotta_controller.CobottaController()
    joint_list = [0, 0, 0, 0, 0, 0]
    saved_pos_label_list = []
    saved_joint_list = []
    saved_pos_list = []


    def main(page: ft.Page):
        page.title = "COBOTTA Controller"
        page.window_width = 470
        page.window_height = 1040

        def update_joint():
            now_joint = cobotta.get_now_joint()
            for i in range(6):
                joint_list[i] = now_joint[i]
            text_j1.value = "{:.2f}".format(joint_list[0])
            text_j2.value = "{:.2f}".format(joint_list[1])
            text_j3.value = "{:.2f}".format(joint_list[2])
            text_j4.value = "{:.2f}".format(joint_list[3])
            text_j5.value = "{:.2f}".format(joint_list[4])
            text_j6.value = "{:.2f}".format(joint_list[5])

        def update_pos():
            pos = cobotta.get_now_position()
            text_pos_x.value = "{:.2f}".format(pos[0])
            text_pos_y.value = "{:.2f}".format(pos[1])
            text_pos_z.value = "{:.2f}".format(pos[2])
            text_pos_rx.value = "{:.2f}".format(pos[3])
            text_pos_ry.value = "{:.2f}".format(pos[4])
            text_pos_rz.value = "{:.2f}".format(pos[5])

        def button_plus_minus_clicked(e):
            update_joint()
            update_pos()
            if e.control.data[0] == 'J':
                if e.control.data[2] == 'plus':
                    joint_list[e.control.data[1]] += slider_step.value
                elif e.control.data[2] == 'minus':
                    joint_list[e.control.data[1]] -= slider_step.value
                cobotta.go_to_joint_parameter(joint_list)
            elif e.control.data[0] == 'P':
                shift = [0, 0, 0, 0, 0, 0]
                if e.control.data[2] == 'plus':
                    shift[e.control.data[1]] = slider_shift_step.value
                elif e.control.data[2] == 'minus':
                    shift[e.control.data[1]] = -slider_shift_step.value
                cobotta.shift_parallel(x=shift[0], y=shift[1], z=shift[2], rx=shift[3], ry=shift[4], rz=shift[5])
                now_joint = cobotta.get_now_joint()
                for i in range(6):
                    joint_list[i] = now_joint[i]
            update_joint()
            update_pos()
            page.update()

        def limit_enable_changed(e):
            if e.control.data == 0:
                if e.control.value:
                    slider_step.max = 50
                else:
                    slider_step.max = 10
                    slider_step.value = 10
                    text_step.value = "10.0deg"
            elif e.control.data == 1:
                if e.control.value:
                    slider_shift_step.max = 50
                else:
                    slider_shift_step.max = 10
                    slider_shift_step.value = 10
                    text_shift_step.value = "10.0mm"
            page.update()


        # General control
        def button_connect_clicked(e):
            cobotta.connect()
            connect_status.value = "Connected!"
            cobotta.change_speed(speed=50)
            slider_speed.value = 50
            slider_step.value = 1
            slider_shift_step.value = 1
            text_speed.value = "{}%".format(int(slider_speed.value))
            text_step.value = "{:.1f}deg".format(float(slider_step.value))
            text_shift_step.value = "{:.1f}mm".format(float(slider_shift_step.value))
            now_joint = cobotta.get_now_joint()
            for i in range(6):
                joint_list[i] = now_joint[i]
            update_joint()
            update_pos()
            page.update()

        def button_disconnect_clicked(e):
            cobotta.disconnect()
            connect_status.value = "Disconnected!"
            page.update()

        def button_calibration_clicked(e):
            cobotta.auto_calibration()
            # now_joint = cobotta.get_now_joint()
            # for i in range(6):
            #     joint_list[i] = now_joint[i]
            # update_joint()
            # update_pos()
            connect_status.value = "Disconnected!"
            page.update()

        def button_clear_error_clicked(e):
            cobotta.clear_error()
            cobotta.motion_preparation()
            page.update()

        def reload_clicked(e):
            update_joint()
            update_pos()
            page.update()

        def motor_changed(e):
            cobotta.turn_off_motor()
            page.update()

        def go_to_sleep_position(e):
            cobotta.go_to_sleep_position()
            now_joint = cobotta.get_now_joint()
            for i in range(6):
                joint_list[i] = now_joint[i]
            update_joint()
            update_pos()
            page.update()

        def open_door(e):
            cobotta.open_door()
            page.update()

        def close_door(e):
            cobotta.close_door()
            page.update()

        def open_hand(e):
            cobotta.open_hand()
            page.update()

        def close_hand(e):
            cobotta.close_hand()
            page.update()

        button_connect = ft.Row(controls=[ft.ElevatedButton(text="Connect", on_click=button_connect_clicked)])
        button_disconnect = ft.Row(controls=[ft.ElevatedButton(text="Disconnect", on_click=button_disconnect_clicked)])
        connect_status = ft.Text("")
        button_calibration = ft.Row(controls=[ft.ElevatedButton(text="Calibration", on_click=button_calibration_clicked)])
        button_clear_error = ft.Row(controls=[ft.ElevatedButton(text="Clear error", on_click=button_clear_error_clicked, icon=ft.icons.ERROR_OUTLINE)])
        button_motor = ft.Row(controls=[ft.ElevatedButton(text="Turn off motor", on_click=motor_changed)])
        button_go_to_sleep_position = ft.Row(controls=[ft.ElevatedButton(text="Go to sleep position", on_click=go_to_sleep_position, icon=ft.icons.HOME)])
        button_open_door = ft.Row(controls=[ft.ElevatedButton(text="Open door", on_click=open_door)])
        button_close_door = ft.Row(controls=[ft.ElevatedButton(text="Close door", on_click=close_door)])
        button_open_hand = ft.Row(controls=[ft.ElevatedButton(text="Open hand", on_click=open_hand)])
        button_close_hand = ft.Row(controls=[ft.ElevatedButton(text="Close hand", on_click=close_hand)])
        button_reload = ft.Row(controls=[ft.ElevatedButton(text="Reload", on_click=reload_clicked)])

        page.add(ft.Row(controls=[button_connect, button_disconnect, connect_status]))
        page.add(ft.Row(controls=[button_calibration,button_clear_error, button_motor]))
        page.add(ft.Row(controls=[button_go_to_sleep_position, button_reload]))
        page.add(ft.Row(controls=[button_open_door, button_close_door]))
        page.add(ft.Row(controls=[button_open_hand, button_close_hand]))


        # Speed control
        def slider_speed_changed(e):
            text_speed.value = "{}%".format(int(e.control.value))
            cobotta.change_speed(speed=int(e.control.value))
            page.update()

        slider_speed = ft.Slider(min=1, max=100, divisions=20, label="{value}%", on_change_end=slider_speed_changed)
        text_speed = ft.Text("")
        speed = ft.Row(
            [
                ft.Text("Speed"),
                slider_speed,
                text_speed
            ]
        )

        page.add(speed)


        # Joint control
        def slider_step_changed(e):
            text_step.value = "{:.1f}deg".format(float(e.control.value))
            page.update()

        slider_step = ft.Slider(min=0.1, max=10, divisions=100, label="{value}deg", on_change_end=slider_step_changed)
        text_step = ft.Text("")
        button_limit_enable = ft.Checkbox(label="Limit enable", on_change=limit_enable_changed, data=0)
        step = ft.Row(
            [
                ft.Text("Step"),
                slider_step,
                text_step,
                button_limit_enable
            ]
        )

        text_j1 = ft.Text("")
        joint1 = ft.Row(
            [
                ft.Text("J1"),
                ft.IconButton(ft.icons.ROTATE_90_DEGREES_CW_OUTLINED, on_click=button_plus_minus_clicked, data=['J', 0, 'minus']),
                ft.Slider(min=-150, max=150, divisions=1000, label="{value}%", disabled=True),
                ft.IconButton(ft.icons.ROTATE_90_DEGREES_CCW_SHARP, on_click=button_plus_minus_clicked, data=['J', 0, 'plus']),
                text_j1
            ]
        )
        text_j2 = ft.Text("")
        joint2 = ft.Row(
            [
                ft.Text("J2"),
                ft.IconButton(ft.icons.REMOVE, on_click=button_plus_minus_clicked, data=['J', 1, 'minus']),
                ft.Slider(min=-60, max=100, divisions=1000, label="{value}%", disabled=True),
                ft.IconButton(ft.icons.ADD, on_click=button_plus_minus_clicked, data=['J', 1, 'plus']),
                text_j2
            ]
        )
        text_j3 = ft.Text("")
        joint3 = ft.Row(
            [
                ft.Text("J3"),
                ft.IconButton(ft.icons.REMOVE, on_click=button_plus_minus_clicked, data=['J', 2, 'minus']),
                ft.Slider(min=18, max=140, divisions=1000, label="{value}%", disabled=True),
                ft.IconButton(ft.icons.ADD, on_click=button_plus_minus_clicked, data=['J', 2, 'plus']),
                text_j3
            ]
        )
        text_j4 = ft.Text("")
        joint4 = ft.Row(
            [
                ft.Text("J4"),
                ft.IconButton(ft.icons.REMOVE, on_click=button_plus_minus_clicked, data=['J', 3, 'minus']),
                ft.Slider(min=-170, max=170, divisions=1000, label="{value}%", disabled=True),
                ft.IconButton(ft.icons.ADD, on_click=button_plus_minus_clicked, data=['J', 3, 'plus']),
                text_j4
            ]
        )
        text_j5 = ft.Text("")
        joint5 = ft.Row(
            [
                ft.Text("J5"),
                ft.IconButton(ft.icons.REMOVE, on_click=button_plus_minus_clicked, data=['J', 4, 'minus']),
                ft.Slider(min=-95, max=135, divisions=1000, label="{value}%", disabled=True),
                ft.IconButton(ft.icons.ADD, on_click=button_plus_minus_clicked, data=['J', 4, 'plus']),
                text_j5
            ]
        )
        text_j6 = ft.Text("")
        joint6 = ft.Row(
            [
                ft.Text("J6"),
                ft.IconButton(ft.icons.REMOVE, on_click=button_plus_minus_clicked, data=['J', 5, 'minus']),
                ft.Slider(min=-170, max=170, divisions=1000, label="{value}%", disabled=True),
                ft.IconButton(ft.icons.ADD, on_click=button_plus_minus_clicked, data=['J', 5, 'plus']),
                text_j6
            ]
        )

        page.add(step)
        page.add(joint1)
        page.add(joint2)
        page.add(joint3)
        page.add(joint4)
        page.add(joint5)
        page.add(joint6)


        # Parallel control
        def slider_shift_step_changed(e):
            text_shift_step.value = "{:.1f}mm".format(float(e.control.value))
            page.update()

        slider_shift_step = ft.Slider(min=0.1, max=10, divisions=100, label="{value}mm", on_change_end=slider_shift_step_changed)
        text_shift_step = ft.Text("")
        button_limit_enable_shift = ft.Checkbox(label="Limit enable", on_change=limit_enable_changed, data=1)
        shift_step = ft.Row(
            [
                ft.Text("Shift step"),
                slider_shift_step,
                text_shift_step,
                button_limit_enable_shift
            ]
        )
        text_pos_x = ft.Text("")
        text_pos_y = ft.Text("")
        text_pos_z = ft.Text("")
        text_pos_rx = ft.Text("")
        text_pos_ry = ft.Text("")
        text_pos_rz = ft.Text("")
        row_1 = ft.Row(
            [
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.IconButton(ft.icons.ARROW_UPWARD, on_click=button_plus_minus_clicked, data=['P', 0, 'minus']),
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.IconButton(ft.icons.KEYBOARD_DOUBLE_ARROW_UP, on_click=button_plus_minus_clicked, data=['P', 2, 'plus']),
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.Text("X"),
                text_pos_x,
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000", icon_size=0),
                ft.Text("RX"),
                text_pos_rx            
            ]
        )
        row_2 = ft.Row(
            [
                ft.IconButton(ft.icons.ARROW_BACK, on_click=button_plus_minus_clicked, data=['P', 1, 'minus']),
                ft.IconButton(ft.icons.EXPAND_CIRCLE_DOWN_ROUNDED, disabled=True),
                ft.IconButton(ft.icons.ARROW_FORWARD, on_click=button_plus_minus_clicked, data=['P', 1, 'plus']),
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.Text("Y"),
                text_pos_y,
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000", icon_size=0),
                ft.Text("RY"),
                text_pos_ry
            ]
        )
        row_3 = ft.Row(
            [
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.IconButton(ft.icons.ARROW_DOWNWARD, on_click=button_plus_minus_clicked, data=['P', 0, 'plus']),
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.IconButton(ft.icons.KEYBOARD_DOUBLE_ARROW_DOWN, on_click=button_plus_minus_clicked, data=['P', 2, 'minus']),
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000"),
                ft.Text("Z"),
                text_pos_z,
                ft.IconButton(ft.icons.ARROW_UPWARD, disabled=True, icon_color="#00000000", icon_size=0),
                ft.Text("RZ"),
                text_pos_rz
            ]
        )

        page.add(shift_step)
        page.add(row_1)
        page.add(row_2)
        page.add(row_3)

        # Demo control
        def dropdown_changed(e):
            if e.control.value == "Tower→Platform" or e.control.value == "MiniFlex→Tower" or e.control.value == "pPMp":
                dropdown_floor.disabled = False
                dropdown_room.disabled = False
            else:
                dropdown_floor.disabled = True
                dropdown_room.disabled = True
            button_demo_run.disabled = False
            page.update()

        def button_demo_run_clicked(e):
            if dropdown_demo.value == "Tower→Platform":
                cobotta.transport_from_tower_to_platform(floor=int(dropdown_floor.value), room=int(dropdown_room.value))
            elif dropdown_demo.value == "Platform→MiniFlex":
                cobotta.transport_from_platform_to_miniflex()
            elif dropdown_demo.value == "MiniFlex→Tower":
                cobotta.transport_from_miniflex_to_tower(floor=int(dropdown_floor.value), room=int(dropdown_room.value))
            elif dropdown_demo.value == "All":
                # time.sleep(10)
                cobotta.transport_from_tower_to_platform(floor=0, room=0)
                cobotta.transport_from_platform_to_miniflex()
                cobotta.transport_from_miniflex_to_tower(floor=0, room=0)
            elif dropdown_demo.value == "mount":
                cobotta.attach_paper()
                cobotta.go_to_position_parameter(cobotta.positions["before_mounting"])
                cobotta.go_to_position_parameter(cobotta.positions["center_of_platform"])
                cobotta.mount_sample(cobotta.positions["mount_center"],0.5,1)
                cobotta.detach_paper()
                cobotta.go_to_sleep_position()
            # elif dropdown_demo.value == "pPMp":
            #     # tsukeru
            #     cobotta.open_door()
            #     cobotta.go_to_joint_parameter([50.01, 80.91, 94.85, 0.01, -85.73, -90.04])
            #     cobotta.change_speed(20)
            #     cobotta.shift_parallel(z=-11)
            #     time.sleep(0.5)
            #     cobotta.shift_parallel(z=11)
            #     cobotta.change_speed()

            #     # cobotta.transport_from_platform_to_miniflex()

            #     cobotta.open_door()
            #     cobotta.close_funnel()
            #     cobotta.go_to_position_parameter(cobotta.positions["before_mounting"])
            #     cobotta.go_to_position_parameter(cobotta.positions["center_of_platform"])
            #     cobotta.mount_sample(cobotta.positions["mount_center"],0.5,1)
            #     cobotta.shift_parallel(y=-40)
            #     cobotta.go_to_position_parameter(cobotta.positions["in_front_of_platform"])
            #     cobotta.go_to_position_parameter(cobotta.positions["above_platform"])
            #     cobotta.shift_parallel(z=5)
            #     cobotta.close_hand()
            #     time.sleep(0.5)
            #     cobotta.open_hand()
            #     cobotta.shift_parallel(z=-3)
            #     cobotta.go_to_position_parameter_at_P(cobotta.positions["in_front_of_platform"])
            #     cobotta.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
            #     for i in (0,1,2):
            #         cobotta.go_to_position_parameter_at_P(cobotta.positions["in_miniflex"][i])
            #     cobotta.change_speed(10,50,100)
            #     for i in (3,4):
            #         cobotta.go_to_position_parameter(cobotta.positions["in_miniflex"][i])
            #     time.sleep(1)
            #     cobotta.go_to_position_parameter(cobotta.positions["in_miniflex"][3])
            #     cobotta.change_speed(100,100,100)
            #     cobotta.go_to_position_parameter(cobotta.positions["in_miniflex"][2])
            #     for i in (1,0):
            #         cobotta.go_to_position_parameter_at_P(cobotta.positions["in_miniflex"][i])
            #     cobotta.go_to_joint_parameter([18.39087487348178, 0.9160677967274204, 69.41830248122903, 4.853530266116941, 19.73065564704061, -94.57060277605763])
            #     cobotta.go_to_position_parameter(cobotta.positions["before_miniflex"])

            #     # hazusu
            #     cobotta.go_to_joint_parameter([0.0, 34.96, 122.21, -3.79, -67.08, -88.49])
            #     cobotta.go_to_joint_parameter([-23.58, 34.93, 122.21, -3.8, -67.06, -88.49])
            #     cobotta.change_speed(15)
            #     cobotta.shift_parallel(z=-20)
            #     time.sleep(0.2)
            #     cobotta.shift_parallel(z=20)
            #     cobotta.change_speed()
            #     cobotta.move_joint1(23.58)
            #     cobotta.go_to_sleep_position()
            #     cobotta.close_door()

            page.update()

        dropdown_floor = ft.Dropdown(
            width=70,
            height=50,
            text_size=15,
            disabled=True,
            label="Floor",
            value="0",
            options=[
                ft.dropdown.Option("0"),
                ft.dropdown.Option("1"),
                ft.dropdown.Option("2"),
                ft.dropdown.Option("3"),
                ft.dropdown.Option("4"),
                ft.dropdown.Option("5")
            ]
        )
        dropdown_room = ft.Dropdown(
            width=70,
            height=50,
            text_size=15,
            disabled=True,
            label="Room",
            value="0",
            options=[
                ft.dropdown.Option("0"),
                ft.dropdown.Option("1")
            ]
        )
        dropdown_demo = ft.Dropdown(
            on_change=dropdown_changed,
            width=175,
            height=50,
            text_size=15,
            label="Demo",
            options=[
                ft.dropdown.Option("Tower→Platform"),
                ft.dropdown.Option("Platform→MiniFlex"),
                # ft.dropdown.Option("pPMp"),
                ft.dropdown.Option("MiniFlex→Tower"),
                ft.dropdown.Option("All"),
                ft.dropdown.Option("mount")
            ]
        )
        button_demo_run = ft.ElevatedButton(text="Demo Run", disabled=True, on_click=button_demo_run_clicked)

        page.add(ft.Row([dropdown_demo, dropdown_floor, dropdown_room, button_demo_run]))


        # Save position
        def button_save_pos_clicked(e):
            if textfield_pos_label.value == "xxx":
                saved_pos_label_list.clear()
                saved_joint_list.clear()
                saved_pos_list.clear()
                dropdown_pos.options.clear()
                textfield_pos_label.value = ""
                print("Saved position list cleared")
                page.update()
                return
            elif textfield_pos_label.value == "outl":
                textfield_pos_label.value = ""
                page.update()
                print("=====================================")
                print("label_list=",saved_pos_label_list)
                print("joint_list=: ",saved_joint_list)
                print("pos_list: ",saved_pos_list)
                print("=====================================")
                return
            elif textfield_pos_label.value == "outp":
                textfield_pos_label.value = ""
                page.update()
                print("=====================================")
                for i in range(len(saved_pos_label_list)):
                    print(saved_pos_label_list[i], " ", saved_pos_list[i])
                print("=====================================")
                return
            elif textfield_pos_label.value == "outj":
                textfield_pos_label.value = ""
                page.update()
                print("=====================================")
                for i in range(len(saved_pos_label_list)):
                    print(saved_pos_label_list[i], " ", saved_joint_list[i])
                print("=====================================")
                return

            elif textfield_pos_label.value in saved_pos_label_list:
                count = 2
                while True:
                    if textfield_pos_label.value + "_" + str(count) in saved_pos_label_list:
                        count += 1
                    else:
                        textfield_pos_label.value = textfield_pos_label.value + "_" + str(count)
                        break
            elif textfield_pos_label.value == "":
                count = 1
                while True:
                    if "Pos_" + str(count) in saved_pos_label_list:
                        count += 1
                    else:
                        textfield_pos_label.value = "Pos_" + str(count)
                        break
            now_pos = cobotta.get_now_position()
            now_pos = [round(i, 2) for i in now_pos]
            saved_pos_list.append(now_pos)
            now_joint = cobotta.get_now_joint()
            now_joint = [round(i, 2) for i in now_joint]
            saved_joint_list.append(now_joint)
            saved_pos_label_list.append(textfield_pos_label.value)
            print("Saved position: ", textfield_pos_label.value, " ", now_joint)
            dropdown_pos.options.append(ft.dropdown.Option(textfield_pos_label.value))
            textfield_pos_label.value = ""
            page.update()
        
        def button_goto_pos_clicked(e):
            index = saved_pos_label_list.index(dropdown_pos.value)
            cobotta.go_to_joint_parameter(saved_joint_list[index])
            print("Go to ", saved_pos_label_list[index])
            update_joint()
            update_pos()
            page.update()

        textfield_pos_label = ft.TextField(label="Name", width=100, height=50, text_size=15)
        button_save_pos = ft.ElevatedButton(text="Save", on_click=button_save_pos_clicked)
        dropdown_pos = ft.Dropdown(
            width=150,
            height=50,
            text_size=15,
            label="Saved position",
            options=[]
        )
        button_goto_pos = ft.ElevatedButton(text="Go", on_click=button_goto_pos_clicked)

        page.add(ft.Row([textfield_pos_label, button_save_pos, dropdown_pos, button_goto_pos]))

    ft.app(target=main)