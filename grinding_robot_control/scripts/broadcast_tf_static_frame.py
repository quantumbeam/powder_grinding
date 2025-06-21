#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations 

class FTSensorFrameBroadcaster:
    """
    静的なTF（座標変換）をブロードキャストするクラス。
    インスタンス化された後、broadcast_transformメソッドを呼び出すことで
    TFを発行します。
    """
    def __init__(self):
        """
        コンストラクタ。StaticTransformBroadcasterを初期化する。
        """
        # StaticTransformBroadcasterは一度だけ初期化すれば良い
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.loginfo("FTSensorFrameBroadcaster: initialized.")

    def broadcast_transform(self, parent_frame_id, child_frame_id, translation, rotation_euler_rpy=(0, 0, 0)):
        """
        指定されたパラメータで静的なTFをブロードキャストする。

        Args:
            parent_frame_id (str): 親フレームID
            child_frame_id (str): 子フレームID
            translation (list[float] or tuple[float]): 並進ベクトル [x, y, z]
            rotation_euler_rpy (list[float] or tuple[float], optional):
                回転 (オイラー角 Roll, Pitch, Yaw [rad])。デフォルトは (0, 0, 0)。
        """
        if not isinstance(translation, (list, tuple)) or len(translation) != 3:
            rospy.logerr("Translation must be a list or tuple of 3 floats [x, y, z].")
            return
        if not isinstance(rotation_euler_rpy, (list, tuple)) or len(rotation_euler_rpy) != 3:
            rospy.logerr("Rotation must be a list or tuple of 3 floats [roll, pitch, yaw].")
            return

        # TransformStamped メッセージの作成
        static_transform_stamped = geometry_msgs.msg.TransformStamped()

        # ヘッダー情報
        static_transform_stamped.header.stamp = rospy.Time.now() # 送信時のタイムスタンプ
        static_transform_stamped.header.frame_id = parent_frame_id      # 親フレームID
        static_transform_stamped.child_frame_id = child_frame_id # 子フレームID

        # 並進 (Translation)
        static_transform_stamped.transform.translation.x = translation[0]
        static_transform_stamped.transform.translation.y = translation[1]
        static_transform_stamped.transform.translation.z = translation[2]

        # 回転 (Rotation) - オイラー角からクォータニオンへ変換
        q = tf.transformations.quaternion_from_euler(
            rotation_euler_rpy[0], rotation_euler_rpy[1], rotation_euler_rpy[2]
        )
        static_transform_stamped.transform.rotation.x = q[0]
        static_transform_stamped.transform.rotation.y = q[1]
        static_transform_stamped.transform.rotation.z = q[2]
        static_transform_stamped.transform.rotation.w = q[3]

        # 静的TFをブロードキャスト
        # sendTransformはリストも受け付けるが、単一でも可
        self.broadcaster.sendTransform(static_transform_stamped)

        rospy.loginfo(f"Successfully broadcasted static transform from '{parent_frame_id}' to '{child_frame_id}'.")
        rospy.loginfo(f"  Translation: {translation}")
        rospy.loginfo(f"  Rotation (RPY): {rotation_euler_rpy}")


# --- 以下は、このクラスを使用する例 (元のスクリプトの main 部分を改変) ---
# このファイルが直接実行された場合にのみ、以下のコードが実行される
if __name__ == '__main__':
    # ノードを初期化 (クラスの外で一度だけ行うのが一般的)
    rospy.init_node('static_tf_broadcaster_node') # ノード名は必要に応じて変更

    try:
        # --- パラメータの取得 ---
        # launchファイルで設定されているパラメータ名に合わせて取得
        target_position = rospy.get_param('~'+rospy.get_param('~initial_position_param_name'))
        print(target_position)
        if not isinstance(target_position, dict) or not all(k in target_position for k in ['x', 'y', 'z']):
             raise rospy.ROSException("Parameter '~target_position' is not a valid dictionary with keys 'x', 'y', 'z'")
        parent_frame_id = rospy.get_param('~parent_frame_id', 'base_link')
        child_frame_id = rospy.get_param('~child_frame_id', 'ft_sensor_base')
        translation_offset=rospy.get_param('~translation_offset',[0.0, 0.0, 0.0])
        rotation_rpy_offset=rospy.get_param('~rotation_rpy_offset',[0.0, 0.0, 0.0])
        # --- 型チェック ---
        if not isinstance(target_position, dict) or not all(k in target_position for k in ['x', 'y', 'z']):
             raise rospy.ROSException("Parameter '~target_position' is not a valid dictionary with keys 'x', 'y', 'z'")
        if not isinstance(parent_frame_id, str):
            raise rospy.ROSException(f"Parameter '~parent_frame_id' must be a string, but got {type(parent_frame_id)}")
        if not isinstance(child_frame_id, str):
            raise rospy.ROSException(f"Parameter '~child_frame_id' must be a string, but got {type(child_frame_id)}")
        if not isinstance(translation_offset, list) or len(translation_offset) != 3:
            raise rospy.ROSException(f"Parameter '~translation_offset' must be a list of 3 floats, but got {type(translation_offset)}")
        if not isinstance(rotation_rpy_offset, list) or len(rotation_rpy_offset) != 3:
            raise rospy.ROSException(f"Parameter '~rotation_rpy_offset' must be a list of 3 floats, but got {type(rotation_rpy_offset)}")

        # 型チェックが通ったら、値を展開
        target_position = [target_position["x"], target_position["y"], target_position["z"]]

        # パラメータ読み込み成功のログ（型チェック後）
        rospy.loginfo(f"Parameters loaded and validated:")
        rospy.loginfo(f"  parent_frame_id: {parent_frame_id}")
        rospy.loginfo(f"  child_frame_id: {child_frame_id}")
        rospy.loginfo(f"  target_position (base): {target_position}")
        rospy.loginfo(f"  translation_offset: {translation_offset}")
        rospy.loginfo(f"  rotation_rpy_offset: {rotation_rpy_offset}")


        # --- クラスの利用 ---
        # FTSensorFrameBroadcaster: クラスのインスタンスを作成
        tf_broadcaster = FTSensorFrameBroadcaster()

        # broadcast_transform メソッドに必要なデータを準備
        # z座標にオフセットを適用
        translation_vector = [    target_position[0] + translation_offset[0],
            target_position[1] + translation_offset[1],
            target_position[2] + translation_offset[2]
        ]
        # 回転は Roll=0, Pitch=0, Yaw=yaw_rotation
        rotation_rpy = (
            rotation_rpy_offset[0],
            rotation_rpy_offset[1],
            rotation_rpy_offset[2]
        )

        rospy.loginfo("Broadcasting static transform...")

        # TFをブロードキャスト
        tf_broadcaster.broadcast_transform(
            parent_frame_id,
            child_frame_id,
            translation_vector,
            rotation_rpy
        )

        # ノードが終了しないように spin() を呼び出す
        # StaticTransformBroadcasterは一度送信すればよいため、
        # このノードが他に何もしないのであれば spin() は必須
        rospy.loginfo("Static TF broadcast complete. Spinning to keep node alive.")
        rospy.spin()

    except (rospy.ROSException, KeyError) as e:
        rospy.logerr(f"Failed to initialize or broadcast TF: {e}")
        exit(1)
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
        exit(1)

