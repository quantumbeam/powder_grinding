## ROSのaudio_common
- 音の取得と再生、またtopicでのpublishなどを行えるパッケージ
- 再生側(audio_play)はpythonとc++の両方が用意されててそれなりに充実している
- 録音側(audio_record)は需要がないのかc++しか用意されておらず、あまり活発ではない印象

## audio_record
- gstreamerというcで記述されているマルチメディアフレームワークを使ってc++で実装
- [チュートリアル](http://wiki.ros.org/audio_common/Tutorials/Streaming%20audio)でgst-launchでチェックしてねって項目がでてくるが、エラーも起きやすく初めて触る人には意味が分からないので簡単に紹介
  - [ここが参考になった](http://www.katsuster.net/index.php?arg_act=cmd_show_diary&arg_date=20130913)


## ISSUE
- gst-launch-1.0 alsasrc ! audioconvert ! audioresample ! alsasink が動作しない
  - デバイス指定で gst-launch-1.0 alsasrc device=hw:2,0 ! audioconvert ! audioresample ! alsasink device=hw:2,0 だと問題なく動作する
  - [ここ](https://mint.hateblo.jp/entry/2018/01/09/024358)に「 ALSA の使える Linux 環境であれば Docker の --device オプションで --device=/dev/snd:/dev/snd としたりすれば動くらしい」とあるが、-- volumes=/dev:/dev だけで問題なく動いた
- roslaunch audio_capture capture.launch が動かない
  - まだ未解決(9/28現在)
  - ここが非常に参考になりそう
    - https://lucasw.github.io/ros-sound/