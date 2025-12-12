# Vstone台車のオプションのパッケージ

## 対応オプション
1. LRF TG30（メガローバー：前後）
2. バンパーセンサ（メガローバー：前後）
3. デプスカメラ　Realsense D435i
4. カメラステー（デプスカメラ固定用）

### カメラステーの高さ、角度を調整したい場合
[cam_stay_mega3.xacro](./urdf/cam_stay/cam_stay_mega3.xacro), [cam_stay_f120a.xacro](./urdf/cam_stay/cam_stay_f120a.xacro)ファイルの3~6行目の`cam_frame_height`、`cam_pitch_angle`の値を変更してください。

### LRFのlaserをgazebo上で可視化したい場合
vs_rover_options_descriptionパッケージのxacroファイル([tg30_mega3.gazebo](./urdf/lrf/tg30_mega3.gazebo), [tg30_f120a.gazebo](./urdf/lrf/tg30_f120a.gazebo))の5~6行目`front_laser_visual`, `back_laser_visual`をtrueに設定してださい。