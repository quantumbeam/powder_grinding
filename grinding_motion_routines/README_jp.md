## クラス設計
- moveit_interface.py
  - MoveitInterfaceClass
    - moveitを実行するためのクラス
    - method
      - goto_pose
        - moveeitの実行をposeで指定
      - goto_waypoints
        - moveeitの実行をwaypointsで指定

- motion_routines.py
  - GrindingMotionClass
    - GrindingMotionの生成と実行を行うクラス
    - method
      - create_waypoints
        - waypointsの生成
      - execute_motion_routine
        - waypointsとその前後の動作を含めたroutineを実行
  - GatheringMotionClass
    - GatheringMotionの生成と実行を行うクラス
    - method
      - create_waypoints
        - waypointsの生成
      - execute_motion_routine
        - waypointsとその前後の動作を含めたroutineを実行
  - CameraMotionClass
    - CameraMotionの生成と実行を行うクラス
    - constructorでpsoe指定
    - method
      - goto_pose
        - camera poseに移動
      

