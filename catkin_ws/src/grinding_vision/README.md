## ros serviceについて
- つねにカメラの情報がほしいわけではないので、visionの処理はserviceで実装したい
- servicenの実装で参考にしたサイトのメモ
  - [ごちゃごちゃしている定義がわかりやすくまとめられてる](https://qiita.com/hoshianaaa/items/6d04acb74c8416f6b1d6)
  - [公式,メッセージの定義](http://wiki.ros.org/ja/ROS/Tutorials/CreatingMsgAndSrv)
  - [公式,pythonでの実装](http://wiki.ros.org/ja/ROS/Tutorials/CreatingMsgAndSrv)
  - オライリーの本「プログラミングROS」がpythonで独自のサービス実装してて結構参考になった
  - [公式リファレンス](http://wiki.ros.org/rospy/Overview/Services)
    - オライリーの本の内容も結局ここに全部書いてあった
  
## web cameraをrosで使う
**使用カメラ**
- logicool c980

### 接続確認
- [v412コマンド](https://qiita.com/rizumi/items/f932b3199be353d328da)

### 使用パッケージ
- RealSenseだとROSパッケージがある
- USBのwebカメラだと[libuvc_camera](http://wiki.ros.org/libuvc_camera)のパッケージ
  - 旧パッケージはuvc_cameraでネットだとこのパッケージを使った情報が多いので注意
  - libuvc_cameraはuvc_cameraと互換があるらしい
  - ちなみにuvcはUSB Video Class なのでこの形式のカメラであればこのパッケージで扱えるはず

### libuvc_cameraの設定
- cameraの情報を追加
  - vender,product IDは[lsusb](https://qiita.com/take5249/items/6aaad432584a5e80bf34)で確認
  - [libuvc_cameraの設定参考](https://qiita.com/srs/items/69210692d7f3f56ccfc8)
  - [同じlogicoolのカメラで参考](https://github.com/ros-drivers/libuvc_ros/issues/37)

## cobottaのcamera(canon製)をrosで使う
**使用カメラ**
- canon N10-W02

### 使用パッケージ
- [参考](https://github.com/DENSORobot/denso_robot_ros/issues/38)
- [東大JSKでdenso(主にcobotta)のパッケージを公開しているので参考になりそう](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_denso_robot)
- denso_robot_rosのパッケージに入っているbcap serviceを使って通信できるらしい


## RealsenseをROSで使う

### 露光などのカメラパラメータ設定
- [参考](https://qiita.com/furushchev/items/c003f19563b1ca00bafb)
- `rosrun rqt_reconfigure rqt_reconfigure`でGUIで設定変更が可能
  - 設定変えて直せなくなって困ったときは、RealsenseViewerのHardwareResetを試す

## AR track
- [パッケージ](http://wiki.ros.org/ar_track_alvar)
- [参考](https://robot.isc.chubu.ac.jp/?p=1063)
- noeticの場合、公式のパッケージだとエラーでて動かなかった
  - 代わりに以下のパッケージを使った
  - https://github.com/machinekoder/ar_track_alvar.git

## ICRA2023,IROS2023:斜めからの画像変換について
- 仮定
  - 斜めから見る場合、カメラの向きとしてX軸方向は歪みがないと仮定する
  - つまり、X軸と並行な距離は変換なしで算出できる
  - 一方、Y軸方向は斜めから見た分縮んで見えるので、ある係数をもちいて線形で変換かける
  - 以下の式で補正
    - $\theta$　楕円の角度
      - openCVの楕円フィッティングのアルゴリズムだとY軸がゼロ〜X軸が90度の間の角度(単位:degree)が得られる
      - なお、長軸の補正計算では90[degree]を引いて位相をずらす
    - $\alpha$　X軸方向/Y軸方向の大きさの比(Y軸方向が縮むので普通は1以上)
    - 補正位置の計算式：$1 - (1 - \alpha) * abs(\sin(\theta))$
      - これから得られた補正係数を、openCVの楕円フィッティングのアルゴリズムで得た長軸と短軸にそれぞれかけて補正
    - 