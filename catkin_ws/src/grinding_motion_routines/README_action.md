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
      


## marker

- グラフィカルな基本図形をRvizに表示できる
- c++での例はたくさんあるが、pythonでの例が少なめなので参考情報を記載
- 一つの図形描画にはMarker型、複数の図形描画にはMarkerArray型を使う

**参考**
-  [PythonでROSのRvizに基本図形を描画する簡単な方法](https://www.robotech-note.com/entry/2018/04/15/221524)
   -  pythonでMarkerを使った例
-  [公式のサンプルプログラム](http://docs.ros.org/en/fuerte/api/rviz/html/marker__array__test_8py_source.html)
   -  pythonでMarkerArray使った例
- [ROS講座24 RvisにVisualizationMarkerを表示する](https://qiita.com/srs/items/3d16bf8cc60a364bd783)
  - c++の例

**ポイント**
**C++とpythonのMarkerArrayの扱い**
- MarkerArayはMarkerの配列
- c++だとはじめにresizeで配列の長さ(=マーカーの数)決めて角要素に値入れるのが普通みたい
- pythonだとMarker型で図形作ってMarkerArray型にappendして使う

**Rviz側の操作**
- Marker(もしくはMarkerArray)のpublish後にRviz→displayのAdd→By topic→Marker(MaekerArray)で追加
- Marker TopicはMarkar(MarlerArray)のプログラムで設定したものに合わせる

**Arrowの仕様**
- 単純な図形ではない分少し仕様が複雑
- 指定方法は2種類
  - startとendの座標を指定する
  - poseを指定する
- 僕は主にロボットのposeのデバッグに使うため、poseを指定して使用
- poseを指定する場合の注意点は以下
  - positionはarrowの根本になる
  - 角度0だとX軸と水平な方向の表示になる
    - デバッグではZ軸の負の方向を向いてほしかったのでオイラー角のpitch(Y軸)をpi/2回転させて表示
  - scaleはx:矢印の長さ,y:XY平面における矢印の幅,z:XZ平面における矢印の幅
    - 基本は`scale.x=scale, scale.y=0.1*scale, scale.z=0.1*scale`にすると見やすい

## moveit

- マニピュレータの順運動学(関節角度→位置)や逆運動学(位置→関節角度)計算を行なってくれるツール

**用語**
- Pose
  - positionのXYZと軸の回転角度のXYZのセット
  - 6次元なので6DPoseともいう
  - 回転角度はオイラー角よりもクオータニオンで表すことが多く、その場合XYZWとなる
    - 多分ジンバルロック回避のため
  - 型はgeometry_msgs::Poseとtf:Transformの2種類が使用される[(参考)](https://qiita.com/srs/items/93d7cc671d206a07deae)
- waypoints
  - Poseをリストにしたもの
- path
  - 逆運動学(IK)で計算したwaypointsに対応する角軸の角度のリスト
  - pathを生成する関数だとcompute_cartesian_pathなどが使える
- plan  
  - pathを用いて動作計画すること
- trajectory
  - plan時に生成される軌道
  - Joint trajectoryとRetimeTrajectoryがある
    - Joint trajectory
      - 各軸の情報だけ
    - Retime trajectory
      - 各軸の情報に加えて動作時間の情報を含んだもの
- execute
  - planを実際に実行すること

**moveitでの動作の流れ(python)**
[参考プログラム](https://github.com/Kuwamai/crane_x7_ros/blob/master/crane_x7_examples/scripts/set_waypoint_with_path.py#L68)
1. 動作させたいPoseとwaypointsを作成
   - つまりPoseのリストを作成
2. IKを用いてwaypointsからpathを作成
   - compute_cartesian_pathを使う
3. pathをplanしてtrajectoryを作成
   - retime_trajectoryを使う
4. excuteで実行


**moveitを使う上でのtips**
- waypointsは連続で同じ値にしてはならない
- waypointsの最初と最後を同じ値にしてはならない
- OMPLはwaypointsの初期値点までの移動で使うと良い
  - いきなりcompute_cartesian_path使うとpathの途中で変なposeをとってその後動作負荷になることがある
  - OMPLは変なポーズ取らないので初期値店まではOMPLで移動→その後waypointsに沿って動作が良い

**OMPLを使おうとして失敗した話**
- moveit tutorialのmoveit_commander使ったチュートリアルでOMPL使ってPoseを指定して動作させていたので、それ使ってPTP的に動作させればいけるのでは？と思った
- 1回のPTPごとに時間がかかるランニングするので、動作が重たすぎてだめだった

**ISSUE**
- GOAL_TOLERANCE_VIOLATED のエラー
  - https://github.com/DENSORobot/denso_robot_ros/issues/5
  - stopped_velocity_toleranceを0にする必要があるそうなので、WINCAPSから作成したdescriptionの中にあるdenso_robot_control.yamlの最下部に以下を追加する
  ```
  constraints:
      stopped_velocity_tolerance: 0  # Override default
  ```
- cobottaが動かない
``` 
[ERROR] [1632209059.024218556]: Failed to write (83204231)
[ERROR] [1632209059.027752514]: Automatically change to normal mode.
[ERROR] [1632209059.081131693]:   [1] 1軸:指定位置がソフトウェアリミットオーバです。 (83204231)
```
- [こちら](https://github.com/DENSORobot/denso_robot_ros/issues/12)を参考に解越
- CALSETや梱包姿勢から動かす時は適当な位置まで軸を動かしてから動作開始する
- CALSETや梱包姿勢の時は軸が動作可能範囲ギリギリまで移動しているため、ソフトウェア的に動作可能範囲をオーバーしてしまっていたらしい
  - ちなみにCALSET後は1軸、梱包姿勢後は4軸がソフトウェアリミットオーバした
- ティーチングペンダントで適当な位置まで軸を移動してからROSで実行すれば解決

## TF
- 公式チュートリアルは実用性にかけているので、実際の利用方法が分かりにくい
    - オススメ
        - https://myenigma.hatenablog.com/entry/20130210/1360491625
        - https://www.slideshare.net/kojiterada5/tftf2

### クオータニオン
- ROSでは角度情報を内部的にクオータニオンで扱う
- 計算が大変だが、便利な関数等がいろいろあるので以下参考
  - http://wiki.ros.org/tf2/Tutorials/Quaternions
  - https://ppdr.softether.net/tf-conversion-cpp#xyzq_to_tf
### オイラー角とクオータニオンの使い分け
- 普段の計算において指定するposeはオイラー角でやったほうが計算が楽
- あるposeとあるposeの中間のposeが欲しい時にクオータニオンでposeとposeを補間すると良い
  - tfにquaternion_slerpとう関数があるので、それ使う2つのpose間を簡単にslerpできる

## cobottaのエマージェンシーストップ自動復帰
以下はデンソーに問い合わせたメールより抜粋

**前提条件**
- ROS動作時はcobottaがスレーブモード
- 復帰コマンド送る時は一旦スレーブモードを解除してから送る
- その後再度スレーブモードに戻す

**参考文献**
- Bcapの仕様について
  - (ORiNインストール先)\ORiN2\CAP\b-CAP\CapLib\DENSO\RC8\Doc\b-CAP_Guide_RC8_ja.pdf
    - 表 5-3 使用できるデータ型（P.66）
    - 付録A. CAP関数 ID と CAO インターフェースの対応表（P.69）
    - Githubで公開している人がいた(グレーかもしれないが)
      - https://github.com/quangounet/denso


- 仕様できるコマンドについて
  - BcapServiceで使用できるコマンドは、ORiNプロパイダと共通になっております。
  - ORiNのコマンドは下記に掲載されております。
  - コマンドやパラメータの指定方法は下記手順を参考に、BcapServiceの使用方法に当てはめください。
  - (ORiNインストール先)\ORiN2\CAO\ProviderLib\DENSO\RC8\Doc\RC8_ProvGuide_ja.pdf


**エラークリアの手順**

1. ロボットとbcap_serviceの立ち上げ
```
roslaunch denso_robot_bringup stdcb_bringup.launch sim:=false ip_address:=192.168.0.1
roslaunch bcap_service bcap_service.launch ip_address:=192.168.0.1
```

2. コントローラハンドルを得る
```
rosservice call /bcap_service '{func_id: 3, vntArgs: [{vt: 8, value: "b-CAP"}, {vt: 8, value: "CaoProv.DENSO.VRC"}, {vt: 8, value: "localhost"}, {vt: 8, value: ""}] }'
```
- `HRESULT: 0 vntRet: vt: 19 value: "<controller handle>"`のように得られる

3. ロボットハンドルを得る
```
rosservice call /bcap_service '{func_id: 7, vntArgs: [{vt: 3, value: "<controller handle>"}, {vt: 8, value: "Robot0"}, {vt: 8, value: ""}] }'
```
- コマンド内に2で得たコントローラハンドルを含むので注意
- `HRESULT: 0 vntRet: vt: 19 value: "<robot handle>"`のように得られる

4. ManualResetPreparationとClearErrorを実施
```
rosservice call /bcap_service '{func_id: 64, vntArgs: [{vt: 3, value: "<robot handle>"}, {vt: 8, value: " ManualResetPreparation"}, {vt: 8, value: ""}] }'
rosservice call /bcap_service '{func_id: 17, vntArgs: [{vt: 19, value: "<controller handle>"}, {vt: 8, value: "ClearError"}] }'
```
- COBOTTAの場合、安全規格上ClearErrorする前に事前確認用コマンドManualResetPreparationが必要となります。

 
**関節角度を指定して復帰位置へ移動**

1. スレーブモードの解除（caoRobot.Execute "slvChangeMode",“0”コマンド）
```
rosservice call /bcap_service '{func_id: 64, vntArgs: [{vt: 3, value: "<robot handle>"}, {vt: 8, value: " slvChangeMode "}, {vt: 8, value: "0"}] }'
```
 

2. 各軸角度を指定して動作（caoRobot.Move 1,"J(0,45,90,0,45,0)"コマンド）
```
rosservice call /bcap_service '{func_id: 72, vntArgs: [{vt: 3, value: "<robot handle>"}, {vt: 3, value: "1"}, {vt: 8, value: "J(0,45,90,0,45,0)"}, {vt: 8, value: ""}] }'
```
 
8．スレーブモード(0)にする（caoRobot.Execute "slvChangeMode",“1” コマンド）
```
rosservice call /bcap_service '{func_id: 64, vntArgs: [{vt: 3, value: "<robot handle>"}, {vt: 8, value: " slvChangeMode "}, {vt: 8, value: "1"}] }'
```