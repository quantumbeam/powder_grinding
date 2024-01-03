import bpy
import pathlib
import math

# colladaファイルが置いてあるディレクトリへのフルパス
target_dir = r"../urdf/cobotta"

# colladaファイル（.dae）があるフォルダを設定
collada_dir = pathlib.Path(target_dir)
# colladaファイルをすべて取得
collada_list = list(collada_dir.glob("*.dae"))

# すべてのオブジェクトを削除
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete()

# 変換処理
for collada in collada_list:
    # すべてのオブジェクトを削除
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    # colladaをインポート
    bpy.ops.wm.collada_import(filepath=collada.as_posix())

    # X軸回転の角度を設定 (90度)
    x_rotation_angle = math.radians(-90)

    # すべてのオブジェクトを取得
    objects = bpy.context.selectable_objects

    # X軸回転を適用
    for obj in objects:
        obj.rotation_euler.x += x_rotation_angle

    # daeとしてエクスポート
    stl_name = (collada.parent / f"{collada.stem}.dae").as_posix()
    bpy.ops.wm.collada_export(filepath=stl_name)

    # stlとしてエクスポート
    stl_name = (collada.parent / f"{collada.stem}.stl").as_posix()
    bpy.ops.export_mesh.stl(filepath=stl_name)
