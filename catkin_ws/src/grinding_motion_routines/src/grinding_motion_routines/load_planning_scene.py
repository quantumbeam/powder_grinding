#!/usr/bin/env python3
import sys
import rospy
import roslib
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown


class PlanningScene:
    def __init__(self, move_group):
        self.move_group = move_group
        self.planning_frame = move_group.get_planning_frame()
        self.scene = PlanningSceneInterface()

    def init_planning_scene(self):
        mortar_mesh_file_path = (
            roslib.packages.get_pkg_dir("grinding_descriptions")
            + "/mesh/Rviz/mortar_40mm.stl"
        )
        table_pos = rospy.get_param("~table_position")
        table_scale = rospy.get_param("~table_scale")
        mortar_pos = rospy.get_param("~mortar_position")
        mortar_inner_scale = rospy.get_param("~mortar_inner_scale")
        funnel_pos = rospy.get_param("~funnel_position")
        funnel_scale = rospy.get_param("~funnel_scale")
        self.table_scale = table_scale

        self._add_table(table_scale, table_pos)
        self._add_mortar(mortar_mesh_file_path, mortar_pos)
        if table_pos["z"] < mortar_pos["z"]:  # mortar position is high than table
            self._add_mortar_base(
                mortar_inner_scale, mortar_pos, table_pos, table_scale
            )
        self._add_funnel(funnel_pos, funnel_scale)

    def _add_table(self, table_scale, table_pos):
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = self.planning_frame
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.z = table_pos["z"]
        table_pose.pose.position.z -= table_scale["z"] / 2
        self.scene.add_box(
            "table",
            table_pose,
            size=(table_scale["x"], table_scale["y"], table_scale["z"]),
        )

    def _add_mortar_base(self, mortar_inner_scale, mortar_pos, table_pos, table_scale):
        mortar_base_pose = geometry_msgs.msg.PoseStamped()
        mortar_base_pose.header.frame_id = self.planning_frame
        mortar_base_pose.pose.orientation.w = 1.0
        mortar_base_pose.pose.position.y = mortar_pos["x"]
        mortar_base_pose.pose.position.y = mortar_pos["y"]
        base_hight = mortar_pos["z"] - table_pos["z"]
        mortar_base_pose.pose.position.z = table_pos["z"] + base_hight / 2
        self.scene.add_box(
            "mortar_base",
            mortar_base_pose,
            size=(table_scale["x"], mortar_inner_scale["y"] * 2, base_hight),
        )

    def _add_mortar(self, file_path, mortar_pos):
        mortar_pose = geometry_msgs.msg.PoseStamped()
        mortar_pose.header.frame_id = self.planning_frame
        mortar_pose.pose.position.x = mortar_pos["x"]
        mortar_pose.pose.position.y = mortar_pos["y"]
        mortar_pose.pose.position.z = mortar_pos["z"]
        self.scene.add_mesh("mortar", mortar_pose, file_path)

    def _add_funnel(self, funnel_pos, funnel_scale):
        funnel_pose = geometry_msgs.msg.PoseStamped()
        funnel_pose.header.frame_id = self.planning_frame
        funnel_pose.pose.orientation.w = 1.0
        funnel_pose.pose.position.x = funnel_pos["x"]
        funnel_pose.pose.position.y = funnel_pos["y"]
        funnel_pose.pose.position.z = funnel_scale["z"] / 2
        self.scene.add_cylinder(
            "funnel",
            funnel_pose,
            funnel_scale["z"],
            funnel_scale["x"],
        )


if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node("load_planning_scene")
    move_group = MoveGroupCommander("manipulator")
    planning_scene = PlanningScene(move_group)
    planning_scene.init_planning_scene()
    roscpp_shutdown()
