### Robotic Powder Grinding for Laboratory Automation
<img src="https://github.com/quantumbeam/powder_grinding/blob/main/wiki/grinding_demo.gif?raw=true" alt="UR powder grinding" width="500">

乳棒と乳鉢用いたロボット粉体粉砕のためのROSパッケージです。
シミュレーション(Gazebo)上での動作とロボット実機での動作ができます。

## 目次
- [目次](#目次)
- [概要](#概要)
- [対応ロボット](#対応ロボット)
- [クイックスタート](#クイックスタート)
  - [PCとロボットとDocker環境のセットアップ](#pcとロボットとdocker環境のセットアップ)
  - [Dockerコンテナの立ち上げ](#dockerコンテナの立ち上げ)
  - [Dockerコンテナ内でのROS環境のビルド](#dockerコンテナ内でのros環境のビルド)
  - [モーションのデモ](#モーションのデモ)
- [既知の課題](#既知の課題)
- [Future Work](#future-work)
  - [Citation](#citation)
- [License](#license)

## 概要
**Last Updated:** 2023/10/24  
This repository focuses on the ROS environment for robot control.

## 対応ロボット
- UR5e
- UR3e
- Cobotta

## クイックスタート

### PCとロボットとDocker環境のセットアップ
- [環境セットアップの資料](./env/docker/README_jp.md)を読んで環境セットアップし、終わったらこちらに戻ってきて以下の続きを実行してください。

### Dockerコンテナの立ち上げ
- ターミナル内でのDockerコンテナの立ち上げ
   - `cd ./env && ./RUN-DOCKER-CONTAINER.sh`
- Terminatorによる複数ターミナルの起動とDockerコンテナの立ち上げ
   - `cd ./env && ./LAUNCH-TERMINATOR-TERMINAL.sh`
      - 立ち上げられた複数ターミナルでは`RUN-DOCKER-CONTAINER.sh`が自動実行されている。

### Dockerコンテナ内でのROS環境のビルド
- 初回のみ実行
   - `./INITIAL_SETUP_ROS_ENVIROMENTS.sh`  
- 通常時のビルド
   - `./BUILD_ROS_WORKSPACE.sh`
-  以上のコマンドは`catkin_ws` のディレクトリ内で実行すること(`RUN-DOCKER-CONTAINER.sh`実行時はデフォルトで`catkin_ws`に入っている。)

### モーションのデモ
- UR5e、cobottaの立ち上げと粉砕モーションのデモファイルを用意しています。
- ロボットの立ち上げ
   ```
   roslaunch grinding_robot_bringup ur5e_bringup.launch
   roslaunch grinding_robot_bringup ur3e_bringup.launch
   roslaunch grinding_robot_bringup cobotta_bringup.launch
   ```
   - シミュレーション使う場合は`sim:=true`で立ち上げてください。
- 粉砕モーションの立ち上げ
   ```
   roslaunch grinding_motion_routines ur3e_grinding_demo.launch
   roslaunch grinding_motion_routines ur5e_grinding_demo.launch
   roslaunch grinding_motion_routines cobotta_grinding_demo.launch
   ```
   - コマンド`g`で粉砕の実行準備(g=grinding)、続けて`y`で粉砕実行します。
   - コマンド`G`でヘラによる粉集めの実行準備(g=grinding)、続けて`y`で粉集め実行します。
- 粉砕パラメータの設定
   -  grinding_motion_routinesパッケージ内のconfig内に設定があります。

## 既知の課題
- 通常版Cobottaの3Dモデルファイルの .deaファイルはROSで読めない形式になっています。
   - grinding_descriptionsパッケージ内のcobotta_description_converter.pyを使うことでblenderの.dae形式に変換され、ROSで読めるようになります。ただ、blenderのpythonモジュールであるbpyをインストールして使用してください。

## Future Work
- シミュレータでの粉砕動作
   - 現時点で製作途中です。
- UR内部もしくは外部の力センサを用いて、乳鉢位置の自動調整を行いたいです。
   - grinding_motion_routinesパッケージの`calibrate_mortar_position.launch`に途中まで作ったものがありますが、まだ完成していないので使う場合はスクリプトを読んで書き換えながら使ってください。

### Citation
- [Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science](https://doi.org/10.1109/IROS47612.2022.9981081) (IROS 2022)
```
@InProceedings{RoboticPowderGrindingWithSoftJig,
  Title                    = {Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science},
  Author                   = {Yusaku Nakajima, Masashi Hamaya, Yuta Suzuki, Takafumi Hawai, Felix Von Drigalski, Kazutoshi Tanaka, Yoshitaka Ushiku and Kanta Ono.},
  Booktitle                = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  Year                     = {2022},
  Doi                      = {10.1109/IROS47612.2022.9981081}
}
```
もし興味があれば、以下の関連研究もご覧ください
- [Robotic Powder Grinding with Audio-Visual Feedback for Laboratory Automation in Materials Science](https://ieeexplore.ieee.org/document/10341526) (IROS 2023)
   -  Github pages [here](https://omron-sinicx.github.io/powder-grinding/) 

## License
This repository is under the MIT license. See [LICENSE](./LICENSE) for details.
