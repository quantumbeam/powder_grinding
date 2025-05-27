#!/usr/bin/env python3
import rospy
import roslib
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander


class PlanningScene:
    """
    A class to manage the planning scene for a MoveIt! move group.

    Attributes:
        move_group (MoveGroupCommander): The move group to manage the planning scene for.
        planning_frame (str): The planning frame for the move group.
        scene (PlanningSceneInterface): The interface to the planning scene.
    """

    def __init__(self, move_group):
        """
        The constructor for PlanningScene class.

        Parameters:
            move_group (str or MoveGroupCommander): The name of the move group or a MoveGroupCommander instance.
        """
        if isinstance(move_group, MoveGroupCommander):
            self.move_group = move_group
        elif isinstance(move_group, str):
            self.move_group = MoveGroupCommander(move_group)
        self.planning_frame = self.move_group.get_planning_frame()
        self.scene = PlanningSceneInterface()
        self.scene.remove_world_object("")

    def init_planning_scene(self):
        """
        Initializes the planning scene by adding objects to it based on ROS parameters.
        """
        table_pos = rospy.get_param("~table_position", None)
        table_scale = rospy.get_param("~table_scale", None)
        if table_pos and table_scale:
            self._add_table(table_scale, table_pos)
        else:
            rospy.logwarn("Table parameters not provided")

        mortar_pos = rospy.get_param("~mortar_top_position", None)
        mortar_mesh_file_name = rospy.get_param("~mortar_mesh_file_name", None)
        if mortar_pos and mortar_mesh_file_name:
            mortar_mesh_file_path = (
                roslib.packages.get_pkg_dir("grinding_descriptions")
                + "/mesh/moveit_scene_object/"
                + mortar_mesh_file_name
                + ".stl"
            )
            rospy.loginfo(f"Adding mortar from {mortar_mesh_file_path}")
            self._add_mortar(mortar_mesh_file_path, mortar_pos)
        else:
            rospy.logwarn("Mortar parameters not provided")


    def _add_table(self, table_scale, table_pos):
        """
        Adds a table to the planning scene.

        Parameters:
            table_scale (dict): The scale of the table.
            table_pos (dict): The position of the table.
        """
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = self.planning_frame
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.z = table_pos["z"]
        table_pose.pose.position.z -= table_scale["z"] / 2
        self.scene.add_box(
            "Table",
            table_pose,
            size=(table_scale["x"], table_scale["y"], table_scale["z"]),
        )

    def _add_mortar(self, file_path, mortar_pos):
        """
        Adds a mortar to the planning scene.

        Parameters:
            file_path (str): The path to the mesh file for the mortar.
            mortar_pos (dict): The position of the mortar.
        """
        mortar_pose = geometry_msgs.msg.PoseStamped()
        mortar_pose.header.frame_id = self.planning_frame
        mortar_pose.pose.position.x = mortar_pos["x"]
        mortar_pose.pose.position.y = mortar_pos["y"]
        mortar_pose.pose.position.z = mortar_pos["z"]
        self.scene.add_mesh("Mortar", mortar_pose, file_path)


if __name__ == "__main__":
    rospy.init_node("load_planning_scene")
    move_group_name = rospy.get_param("~move_group_name")
    planning_scene = PlanningScene(MoveGroupCommander(move_group_name))
    planning_scene.init_planning_scene()
