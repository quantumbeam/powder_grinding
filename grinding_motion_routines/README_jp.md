# 粉砕モーションのパラメータ一覧

## 速度制御と加速度制御について

### デフォルトの動作（簡易実装）
- 通常粉砕時の速度は一定であることが望ましいですが、ここでは簡易的な実装としてデフォルトで速度と加速度を0に設定しております
- 図は関節速度のplotですが波打っている様子がわかります（grinding_motion_routines/doc/5D_spline_with_ZEROvolocity_ZEROaccceralation.png）

### 厳密な速度制御を行わない理由
- これは厳密な速度制御を行った場合、特異姿勢付近で粉砕を行った場合にヤコビアンが発散し危険な速度が出る危険性があるためです

### 厳密な速度制御の使用方法
- 厳密な速度制御を行う場合は、JTC_executerでexecute_by_waypointsもしくはexecute_by_joint_trajectory使用時にstrict_velocity_control=Trueにしてください
- ただし危険性については保証ができかねますので自己責任でお使いください
- この場合、より厳密な速度制御が可能となり波打ちもなくなります（grinding_motion_routines/doc/3D_spline_with_volocity_control.png）

- なお、ここで示した図の実験条件はgrinding_motion_routines/doc/motion_config.mdに記載してあります。

## 粉砕パラメータ一覧
| パラメータ                                                     | 型             | 大きさ/意味合い                                                                                                                           |
| :-------------------------------------------------------: | :-----------: | :--------------------------------------------------------------------------------------------------------------------------------: |
| grinding\_pos\_beginning                                   | List\[float\] | \[X座標, Y座標\]。粉砕開始位置の二次元座標。具体的な数値で指定。                                                                                               |
| grinding\_pos\_end                                        | List\[float\] | \[X座標, Y座標\]。粉砕終了位置の二次元座標。具体的な数値で指定。                                                                                               |
| grinding\_center\_pos                                     | List\[float\] | \[X座標, Y座標\]。円弧粉砕の中心位置の二次元座標。直線粉砕では影響小。具体的な数値で指定。                                                                                  |
| grinding\_number\_of\_rotation                            | int           | 粉砕動作の総回転数。正の整数で指定。                                                                                                                 |
| grinding\_sec\_per\_rotation                              | float         | 1回転あたりの時間（秒）。正の小数で指定。grinding\_number\_of\_rotation \* grinding\_sec\_per\_rotation が合計粉砕時間となり、合計 15秒以下 を推奨(モーションプランニングの計算が重たいため)。 |
| grinding\_number\_of\_waypoints\_per\_circle              | int           | 円弧粉砕時の1周あたりの経由点数。直線粉砕時は始点-終点間の分割数。正の整数で指定。値が大きいほど動きが滑らか(50点以上を推奨)。                                                                 |
| grinding\_angle\_scale                                    | float         | 粉砕角度の変化に対するスケール。0(垂直)から1(接線の法線方向)の範囲で指定。0\~0.3が推奨                                                                                  |
| grinding\_rz\_beginning                                    | float         | 粉砕開始時の粉砕モーションの乳鉢深さパラメータ(mm)。具体的な数値で指定。                                                                                             |
| grinding\_rz\_end                                         | float         | 粉砕終了時の粉砕モーションの乳鉢深さパラメータ(mm)。具体的な数値で指定。                                                                                             |
| grinding\_yaw\_bias                                       | float         | 粉砕動作のヨー角バイアス（ラジアン）。rad(pi) は約3.14ラジアン（180度）。                                                                                       |
| grinding\_joint\_difference\_limit\_for\_motion\_planning | float         | モーションプランニングにおける関節角度変化の許容限界（ラジアン）。正の小数で指定。これを超える変化はモーションプランニングができないように制約。                                                           |

## エピサイクロイド可視化ツール (Epicycloid Visualization Tool)

このツールは、エピサイクロイド曲線を描画し、そのパラメータを調整することで軌跡の変化を視覚的に理解するのに役立ちます。

### アクセス方法
*   このリポジトリをクローンまたはダウンロードします。
*   `doc`内の `epicycloid_visualizer.html` ファイルを見つけます。
*   そのHTMLファイルをウェブブラウザで開いてください。
*   例: `file:///path/to/your/grinding_motion_routines/doc/epicycloid_visualizer.html`
