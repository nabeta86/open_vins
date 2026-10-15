# open_vins (mod) — rpng/open_vins の改変版

- 派生元: https://github.com/rpng/open_vins (ブランチ master)
- 派生元コミット: `69488123ed9362dd44b6f28e7f4680abbff1442b`
  (v2.7-20-g6948812, 2025-11-30, "Merge pull request #530 from rpng/android")
- 改変者: Indoor_drone_slam プロジェクト側 (nabeta86)
- 改変版の置き場: https://github.com/nabeta86/open_vins ブランチ `mod`
  (Docker イメージに焼くコミットにはタグ `mod-YYYYMMDD[-suffix]` を打つ。実験は `mod-<topic>` ブランチ)
- 利用側: `Indoor_drone_slam/ubuntu/.env` の `OPENVINS_MOD_COMMIT` でコミットを固定し、イメージ内の
  `/data/openvins_mod/catkin_ws` に **別ワークスペース** としてビルドする。上流の無改変ビルド
  (`/data/openvins/catkin_ws`) と併存させ、A/B 比較に使う。詳細は `Indoor_drone_slam/docs/openvins_mod.md`

## 変更理由

1. ArUco 経路が読むタグを TagSLAM と同じ AprilTag 36h11 にする
   (Indoor_drone_slam 側ではビルド時パッチで当てていたものを、この fork ではコミットに取り込む)
2. SLAM ランドマークごとの 3×3 位置共分散を `points_slam_cov` (sensor_msgs/PointCloud2) で出す
   (地図側 GNG の信頼度に使う。2026-10-15)
3. (予定) 360 度カメラ (Insta360 Air) 対応: 180° 超の魚眼 (double sphere) と正距円筒のカメラモデル、
   方位ベクトルによる特徴表現・三角測量・更新

## 変更点

- `ov_core/src/track/TrackAruco.h`: 辞書 DICT_6X6_1000 → DICT_APRILTAG_36h11、
  OpenCV<4.7 の分岐でサブピクセル補正を有効化
  (`Indoor_drone_slam/ubuntu/docker/patches/openvins-aruco-apriltag36h11.patch` と同一の内容)

- (2026-10-15, ランドマーク共分散) ROS1 のみ。`estimator_config.yaml` のキーは増やさない
  - `ov_msckf/src/core/LandmarkWithCov.h` (新規): id / 表現 / アンカー / p_FinG / 3×3 共分散の構造体
  - `ov_msckf/src/core/VioManager.h`, `VioManagerHelper.cpp`: `get_features_SLAM_with_cov()`。
    ランドマークのパラメータとアンカー clone (外部パラメータをオンライン推定していればそれも) の周辺共分散
    (`StateHelper::get_marginal_covariance`) を、`UpdaterHelper::get_feature_jacobian_representation` と同じ
    ヤコビアンで大域 xyz へ写す (現在値で線形化、FEJ ではない)。アンカーが無い特徴は飛ばす (abort しない)
  - `ov_msckf/src/ros/ROSVisualizerHelper.h/.cpp`: `get_ros_pointcloud_cov()` — PointCloud2 (x y z float32,
    id uint32, cxx cxy cxz cyy cyz czz float32; point_step 40)、frame "global"、stamp は IMU 時計の状態時刻
  - `ov_msckf/src/ros/ROS1Visualizer.h/.cpp`: ROS param `~publish_landmark_cov` (既定 true) で
    `points_slam_cov` を advertise し、購読者がいるときだけ `publish_features()` から出す

## 元との差分

```bash
git fetch upstream
git diff 69488123ed9362dd44b6f28e7f4680abbff1442b..mod
```
