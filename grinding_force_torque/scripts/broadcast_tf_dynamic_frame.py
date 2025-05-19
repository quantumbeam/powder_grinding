#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations # tfパッケージのクォータニオン変換関数を利用

class DynamicFrameBroadcaster:
    """
    動的なTF（座標変換）を定期的にブロードキャストするクラス。
    """
    def __init__(self, parent_frame_id, child_frame_id, translation, rotation_euler_rpy, rate_hz=10):
        """
        コンストラクタ。

        Args:
            parent_frame_id (str): 親フレームID
            child_frame_id (str): 子フレームID
            translation (list[float]): 並進ベクトル [x, y, z]
            rotation_euler_rpy (list[float]): 回転 (オイラー角 Roll, Pitch, Yaw [rad])
            rate_hz (int): ブロードキャストのレート (Hz)
        """
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.parent_frame_id = parent_frame_id
        self.child_frame_id = child_frame_id
        self.translation = translation
        self.rotation_euler_rpy = rotation_euler_rpy
        self.rate = rospy.Rate(rate_hz)

        rospy.loginfo(f"DynamicFrameBroadcaster: initialized to publish TF from '{parent_frame_id}' to '{child_frame_id}' at {rate_hz} Hz.")
        rospy.loginfo(f"  Translation: {self.translation}")
        rospy.loginfo(f"  Rotation (RPY): {self.rotation_euler_rpy}")

    def broadcast_loop(self):
        """
        TFを定期的にブロードキャストするループ。
        """
        while not rospy.is_shutdown():
            # TransformStamped メッセージの作成
            transform_stamped = geometry_msgs.msg.TransformStamped()

            # ヘッダー情報
            transform_stamped.header.stamp = rospy.Time.now() # 送信時のタイムスタンプを更新
            transform_stamped.header.frame_id = self.parent_frame_id
            transform_stamped.child_frame_id = self.child_frame_id

            # 並進 (Translation)
            transform_stamped.transform.translation.x = self.translation[0]
            transform_stamped.transform.translation.y = self.translation[1]
            transform_stamped.transform.translation.z = self.translation[2]

            # 回転 (Rotation) - オイラー角からクォータニオンへ変換
            q = tf.transformations.quaternion_from_euler(
                self.rotation_euler_rpy[0], self.rotation_euler_rpy[1], self.rotation_euler_rpy[2]
            )
            transform_stamped.transform.rotation.x = q[0]
            transform_stamped.transform.rotation.y = q[1]
            transform_stamped.transform.rotation.z = q[2]
            transform_stamped.transform.rotation.w = q[3]

            # TFをブロードキャスト
            self.broadcaster.sendTransform(transform_stamped)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster_node')

    try:
        # パラメータの取得 (static版と同様)
        target_position_param_name = rospy.get_param('~initial_position_param_name')
        target_position_dict = rospy.get_param('~' + target_position_param_name)
        parent_frame_id = rospy.get_param('~parent_frame_id', 'base_link')
        child_frame_id = rospy.get_param('~child_frame_id')
        translation_offset = rospy.get_param('~translation_offset', [0.0, 0.0, 0.0])
        rotation_rpy_offset = rospy.get_param('~rotation_rpy_offset', [0.0, 0.0, 0.0])
        broadcast_rate = rospy.get_param('~broadcast_rate', 10) # Hz

        # 型チェックと値の展開 (static版と同様)
        if not isinstance(target_position_dict, dict) or not all(k in target_position_dict for k in ['x', 'y', 'z']):
             raise rospy.ROSException(f"Parameter '~{target_position_param_name}' is not a valid dictionary with keys 'x', 'y', 'z'")
        # ... (他のパラメータの型チェックも同様に追加可能)

        target_position_list = [target_position_dict["x"], target_position_dict["y"], target_position_dict["z"]]

        translation_vector = [target_position_list[i] + translation_offset[i] for i in range(3)]
        rotation_rpy = tuple(rotation_rpy_offset)

        # DynamicFrameBroadcaster のインスタンスを作成してループを開始
        dynamic_broadcaster = DynamicFrameBroadcaster(parent_frame_id, child_frame_id, translation_vector, rotation_rpy, broadcast_rate)
        dynamic_broadcaster.broadcast_loop()

    except (rospy.ROSException, KeyError) as e:
        rospy.logerr(f"Failed to initialize or broadcast dynamic TF: {e}")
        exit(1)
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
        exit(1)