
# PCの環境構築
1. ソフトウェアのインストール
- Terminatorのインストール
  - 分割して複数のターミナルを使えます。ROSでは複数ターミナルが必要になることが多いので、便利です。
  - ```sudo apt install terminator```
- Dockerのインストール
  - 少し大変かもしれないですが、以下のサイトを参考にDockerをインストールしてください。
  - https://docs.docker.com/engine/install/ubuntu/
- VSCodeのインストール
  - すでに使っているかもしれませんが、VSCodeがおすすめのエディタです。
  - https://code.visualstudio.com/
    - 特に`Remote-Containers`の拡張機能がDocker環境内でVSCodeを開けるので便利です。

2. ポート開放
- ロボットの実機やLAN接続のセンサーを動かす場合は、ファイアウォールの設定を変更してポート開放を行う必要があります。
  - Ubuntuだと`ufw`で簡単に設定ができます。ない場合は```sudo apt install ufw```でインストールしてください。
  - URのデフォルトのポート開放
    - ```sudo ufw allow 50001```
    - ```sudo ufw allow 50002```
    - ```sudo ufw allow 50003```
    - ```sudo ufw allow 50004```
  - Cobottaのポート開放
    - ```sudo ufw allow 5007```

3. ネットワーク設定
- LANアダプターには静的なIPアドレスを設定する必要があります。
  - 本リポのデフォルト設定: ```192.168.56.5```
    - `compose.yaml`で設定されています。

# ロボットの設定
## URとCobotta共通
- 静的IPアドレスを設定してください。
  - This package default of UR :  ```192.168.56.42```
  - This package default of cobotta : ```192.168.56.11```

## Cobotta
- ホストPCのIPアドレスを設定したものに書き換えてください
- 起動兼をEthernetに変更してください
## Universal Robot
- ```external control.urcap```をUniversal robotのタブレットにインストールしてください。設定画面からインストール可能です。
  - urcapは以下のサイトからダウンロード可能です。https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases
- インストール後に、設置設定からホストPCのIPアドレス、ホスト名、ポート(デフォルトで50002)を設定してください。

# Dockerイメージのビルドとコンテナの立ち上げ
- Dockerイメージのビルドは以下のコマンドで行ってください。
  - ```cd ./env/docker && ./BUILD-DOCKER-IMAGE.sh```
- メインディレクトリでDockerコンテナの立ち上げができます。詳細はリポジトリの[README.md](../README.md)内の"Dockerコンテナの立ち上げ"の項目を読んでください。



# 環境情報
## ホストPC
- Ubuntu: 20.04
  - ROS Noetic
- Docker 24.0.6
  - `RUN-DOCKER-CONTAINER.sh`内でdocker composeのwオプションを使っているため、バージョン24.0.0以上が必要になります。 
  
## ロボット
**Cobotta**
  - Software version: 2.16.12
  - MCU version: 
**UR5e**
  - Software version: 5.11.6
