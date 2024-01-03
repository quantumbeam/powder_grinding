## Known Issues
- Cobotta description file(.dae) from [ROSConverter](https://wiki.ros.org/denso_robot_ros/ROSConverter) is not readable for Rviz, but it's readable for blender.
  - Reconverted .dae file by cobotta_description_converter.py, it's readable for Rviz.


## 概論：Rviz(gazebo)で物体を表示するには
**参考**
- [URDF1](https://qiita.com/srs/items/35bbaadd6c4be1e39bb9)
- [URDF2](https://qiita.com/srs/items/77f378230bf856a3625c)
- [xacro1](https://qiita.com/srs/items/9ac7f2f6e47732bb7535)
- [xacro2][https://qiita.com/srs/items/43528d00ee789171367f]
  
**概論**
- URDF(ユニバーサルロボットデフィニションファイル)で定義する
  - ロボットってはいってるけど、別にロボットだけじゃなくて表示するオブジェクトは全部これで書く
- URDFは変数、マクロ、演算などが使えないので全てベタ書きなので大変
  - そこで変数、マクロ、演算などをURDFで使えるようにしたのがxacro
  - 実行時はxacroがURDFに展開される
- Rvizとgazeboだと別のURDFが必要
  - gazeboは物理演算が入るので質量とかいろいろ定義しないといけない

## URDF
- ロボットをリンクとジョイントで表現するファイル形式
  - 原点とするのlink(worldとかbase_linkとか任意)からスタートする
  - 必ずlink -> joint -> link -> joint ・・・の順で書く
  - 例えばsphareってオブジェクトをURDFで表示したいときは、原点をbase_linkとすると、base_link -> sphare_joint_ -> sphare_link と表現できる
  - jointにはtypeがある
    - マニピュレータでよく使うのは以下
    - ただの物体はfixed
    - 角度に制限のある回転はrevolute
    - 移動ロボなどは別のtypeも使う
      - 詳しくは[こちら](http://wiki.ros.org/urdf/XML/joint)

### link
- ロボットのオブジェクトに相当
- 主な設定は以下
  - オブジェクト,対応するstlファイルやboxなど
  - 色

### joint
- ロボットの関節に相当,つまりTF
- 必須要素は以下
  - origin  表示するオブジェクトのポーズ
  - parent  親link名(jointは何というlinkからつながっているのか)
  - child   子link名(jointは何というlinkにつながるのか)
- 以下はrevoluteで必要となる
  - axis    xyzのどの軸に対する回転か
  - limit   回転角度の限界,effortとvelocityという項目はgazeboシミュレータで使う項目なのでRVizだけ使うときは0でok
- ここではマニピュレータを扱っているのでrevolite以外の設定は省略

### xacro
- URDFはベタ書きなので、マクロとか使って楽できる記法
- xacro書いてURDFに展開して使う
- 以下ができるようになる
  - 変数(property)
  - 数式計算(内部でpythonを使用)
  - ifの条件分岐
  - マクロ
  - xacroのインクルード(別xacroファイルのマクロ呼び出し)
  - launch時に外部変数を受け取る(launchのパラメータとして実装可能)