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
2. (予定) SLAM ランドマークごとの 3×3 位置共分散を `points_slam_cov` (sensor_msgs/PointCloud2) で出す
3. (予定) 360 度カメラ (Insta360 Air) 対応: 180° 超の魚眼 (double sphere) と正距円筒のカメラモデル、
   方位ベクトルによる特徴表現・三角測量・更新

## 変更点

- `ov_core/src/track/TrackAruco.h`: 辞書 DICT_6X6_1000 → DICT_APRILTAG_36h11、
  OpenCV<4.7 の分岐でサブピクセル補正を有効化
  (`Indoor_drone_slam/ubuntu/docker/patches/openvins-aruco-apriltag36h11.patch` と同一の内容)

## 元との差分

```bash
git fetch upstream
git diff 69488123ed9362dd44b6f28e7f4680abbff1442b..mod
```
