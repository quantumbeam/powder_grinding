# Powder Grinding Package on ROS noetic

**Repository for Robotic Powder Grinding in Material Science**

## 関連論文
- [Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science](https://doi.org/10.1109/IROS47612.2022.9981081) (IROS 2022)
- [Robotic Powder Grinding with Audio-Visual Feedback for Laboratory Automation in Materials Science](https://omron-sinicx.github.io/powder-grinding/) (IROS 2023)


## 目次
1. [概要](#概要)
2. [対応ロボット](#対応ロボット)
3. [クイックスタート](#クイックスタート)
   - [PCとロボットとDocker環境のセットアップ](#PCとロボットとDocker環境のセットアップ)
   - [Dockerコンテナの立ち上げ](#dockerコンテナの立ち上げ)
   - [Dockerコンテナ内でのros環境のビルド](#dockerコンテナ内でのros環境のビルド)
4. [既知の課題](#既知の課題)
5. [Future Work](#tuture-work)
6. [License](#license)

## 概要
**Last Updated:** 2023/10/24  
This repository focuses on the ROS environment for robot control.

## 対応ロボット
- UR5e
- Cobotta

## クイックスタート

### PCとロボットとDocker環境のセットアップ
- [環境セットアップの資料](./docker/README_jp.md)を読んで環境セットアップし、終わったらこちらに戻ってきて以下の続きを実行してください。

### Dockerコンテナの立ち上げ
- ターミナル内でのDockerコンテナの立ち上げ
   - `./RUN-DOCKER-CONTAINER.sh`
- Terminatorによる複数ターミナルの起動とDockerコンテナの立ち上げ
   - `./LAUNCH-TERMINATOR-TERMINAL.sh`
      - 立ち上げられた複数ターミナルでは`./RUN-DOCKER-CONTAINER.sh`が自動実行されている。

### Dockerコンテナ内でのROS環境のビルド
- 初回のみ実行
   - `./INITIAL_SETUP_ROS_ENVIROMENTS.sh`  
- 通常時のビルド
   - `./BUILD_ROS_WORKSPACE.sh`
-  以上のコマンドは`catkin_ws` のディレクトリ内で実行すること(`./RUN-DOCKER-CONTAINER.sh`実行時はデフォルトで`catkin_ws`に入っている。)

### 粉砕の実行
**現時点でシミュレータは動かず、ロボット実機しか動かせない点に注意してください。**
- UR5e、cobottaの立ち上げと粉砕モーションのデモファイルを用意しています。
- ロボットの立ち上げ
   - `roslaunch grinding_robot_bringup ur5e_bringup.launch `
   - `roslaunch grinding_robot_bringup cobotta_bringup.launch ` 
- 粉砕モーションの立ち上げ
   - `roslaunch grinding_motion_routines ur5e_grinding_demo.launch`
- 粉砕パラメータの設定
   -  grinding_motion_routinesパッケージ内のconfig内に設定があります。URとcobottaの
   デモで使っています。

## 既知の課題
- 通常版Cobottaの3Dモデルファイルの .deaファイルはROSで読めない形式になっています。
   - grinding_descriptionsパッケージ内のcobotta_description_converter.pyを使うことでblenderの.dae形式に変換され、ROSで読めるようになります。ただ、blenderのpythonモジュールであるbpyをインストールして使用してください。

## Future Work
- シミュレータでの粉砕動作
   - 現時点で製作途中です。
- UR内部もしくは外部の力センサを用いて、乳鉢位置の自動調整を行いたいです。
   - grinding_motion_routinesパッケージの`calibrate_mortar_position.launch`に途中まで作ったものがありますが、まだ完成していないので使う場合はスクリプトを読んで書き換えながら使ってください。

## License
This repository is under the MIT license. See [LICENSE](./LICENSE) for details.
