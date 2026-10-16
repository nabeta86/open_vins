# open_vins (ov360) — rpng/open_vins の改変版 (360 度カメラ対応 + ランドマーク共分散)

- 派生元: https://github.com/rpng/open_vins (ブランチ master)
- 派生元コミット: `69488123ed9362dd44b6f28e7f4680abbff1442b`
  (v2.7-20-g6948812, 2025-11-30, "Merge pull request #530 from rpng/android")
- 改変者: Indoor_drone_slam プロジェクト側 (nabeta86)
- 改変版の置き場: https://github.com/nabeta86/open_vins ブランチ `ov360`
  (Docker イメージに焼くコミットにはタグ `ov360-YYYYMMDD[-suffix]` を打つ。実験は `ov360-<topic>` ブランチ。
  2026-10-16 に `mod` から改名 = 「360 度対応」が名前で分かるように。旧タグ `mod-20261015-{base,landmark-cov,360}` は履歴として残す)
- 利用側: `Indoor_drone_slam/ubuntu/.env` の `OPENVINS_360_COMMIT` でコミットを固定し、イメージ内の
  `/data/openvins_360/catkin_ws` に **別ワークスペース** としてビルドする。上流の無改変ビルド
  (`/data/openvins/catkin_ws`) と併存させ、A/B 比較に使う。詳細は `Indoor_drone_slam/docs/openvins_360.md`

## 変更理由

1. ArUco 経路が読むタグを TagSLAM と同じ AprilTag 36h11 にする
   (Indoor_drone_slam 側ではビルド時パッチで当てていたものを、この fork ではコミットに取り込む)
2. SLAM ランドマークごとの 3×3 位置共分散を `points_slam_cov` (sensor_msgs/PointCloud2) で出す
   (地図側 GNG の信頼度に使う。2026-10-15)
3. 360 度カメラ (Insta360 Air) 対応: 180° 超の魚眼 (double sphere) と正距円筒のカメラモデル、
   方位ベクトルによる特徴表現・三角測量・更新 (2026-10-15、当時のブランチ mod-360 = タグ mod-20261015-360)

4. スケール補正 (2026-10-16): 推定器のスケールが実寸からずれたときに、外部の情報 (手動の係数・既知距離・
   TagSLAM などの参照軌跡) でフィルタの状態そのものを相似変換で引き直す。pose_out (出口側) で位置に係数を
   掛ける方式を置き換えた = 以後の `/ov_msckf/*` の出力・traj.txt・ランドマーク・共分散がすべて補正後の単位になる

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

- (2026-10-15, 360 度カメラ) 既存の pinhole (radtan / equidistant) 経路はビット同一のまま。omni モデルのときだけ
  `CamBase::is_omni()` で方位ベクトル経路に入る
  - `ov_core/src/cam/CamBase.h`: 仮想関数 `is_omni / model_name / num_params / focal_equiv / valid / bearing_valid /
    bearing / project / jacobian_project / residual / residual_weights` を追加。**既定実装は既存 3 関数の合成**
    (`project = distort(p/z)`、`bearing = normalize([undistort, 1])`、`jacobian_project = compute_distort_jacobian · d(p/z)/dp`)
  - `ov_core/src/cam/CamDoubleSphere.h` (新規): Usenko 2018。8 要素は `[fx fy cx cy xi alpha 0 0]`
    (Kalibr ds-none の `[xi alpha fu fv pu pv]` は Indoor_drone_slam の kalibr2openvins.py が並べ替える)
  - `ov_core/src/cam/CamEquirect.h` (新規): 正距円筒。`[W/2π, H/π, W/2−0.5, H/2−0.5]`、u 残差の巻き戻し、cos(lat) の白色化
  - `ov_core/src/cam/CamFactory.h` (新規): `distortion_model` (radtan / equidistant / ds / equirect) → モデル。未知の名前は停止
    (上流は radtan に黙って落ちていた)。`ov_msckf/src/core/VioManagerOptions.h` と
    `ov_init/src/init/InertialInitializerOptions.h` の重複パーサは両方これを呼ぶ
  - `ov_core/src/feat/Feature.h/.cpp`, `FeatureDatabase.h/.cpp`: `bearings` (単位方位、cam ごと) を `uvs_norm` と並べて保持
    (omni のときだけ充填、`clean_*` は同期削除)。`update_feature` に方位付きの多重定義
  - `ov_core/src/track/TrackBase.h`: `insert_measurement()` (pinhole は従来どおり undistort + update_feature、omni は方位も、
    方位が定義されない画素は捨てる)。`TrackKLT / TrackDescriptor / TrackAruco / TrackSIM` の挿入箇所はすべてこれを使う
  - `ov_core/src/track/BearingRansac.h` (新規): 単位方位の 8 点法 RANSAC (エピポーラ面からの角距離)。omni のとき
    `cv::findFundamentalMat` の代わりに使う (しきい値 = px / focal_equiv)
  - `ov_core/src/feat/FeatureInitializer.h/.cpp`: `use_bearings` (VioManager が omni 検出で設定、yaml キーではない) のとき
    線形三角測量は保持した方位を使い、ゲートは z ではなく距離 + 「アンカー方位の前方」。Gauss-Newton は球面上
    (`p_FinA = (b0 + a t1 + b t2)/ρ`、接平面残差 [rad]) の `single_gaussnewton_bearing`
  - `ov_msckf/src/update/UpdaterHelper.cpp` `get_feature_jacobian_full`: omni 分岐 (`project` / `jacobian_project` を直接、
    `residual` / `residual_weights` で白色化、`valid()` でない観測は 0 行)。pinhole 分岐は FEJ の癖も含めそのまま
  - `ov_msckf/src/core/VioManager.cpp`: omni 検出 → `use_bearings`、`feat_rep_*` が GLOBAL_3D / ANCHORED_3D 以外なら停止、
    `init_dyn_use` は警告して無効化 (静的初期化のみ)
  - `ov_msckf/src/core/VioManagerHelper.cpp` `retriangulate_active_tracks`: omni は方位と距離ゲート、cam0 への投影は `project`。
    `active_tracks_uvd` の「深度」は omni では距離
  - `ov_msckf/src/ros/ROS1Visualizer.cpp` `loop_intrinsics`: `distortion_model` はモデル名 (plumb_bob / equidistant /
    double_sphere / equirectangular)
  - `ov_core/src/test_cams.cpp` (新規、`rosrun ov_core test_cams`): 射影↔方位の恒等、ヤコビアンの差分検証、
    pinhole の旧新ビット同一、DS の後方点、equirect の巻き戻し、方位 RANSAC、球面上の三角測量
  - `ov_msckf/src/sim/Simulator.cpp`, `ov_init/src/sim/SimulatorInit.cpp`: カメラの複製を型判別 (CamEqui 以外は
    すべて CamRadtan にしていた = DS/equirect のパラメータを k1,k2 として射影) から `make_camera(model_name())` に変更。
    生成・射影は omni なら `bearing` / `project` / `valid` を使う (距離ゲート)。`config/rpng_sim_ds{,0,1,w}` と
    `config/rpng_sim_equirect` は合成データで omni 経路を検証する設定 (Indoor_drone_slam の
    `nodes/tests/ov360_smoke/smoke_sim_omni.sh` が回す)
  - 未対応 (後日): 動的初期化の omni 対応 (`Factor_ImageReprojCalib` は radtan/equi 固定)、`Simulator` の omni、
    equirect の角度視差 (ZUPT は画素視差のまま)、画像端の KLT wrap

- (2026-10-16, スケール補正) pinhole/omni を問わず、何も指定しなければ従来どおり (既定で状態は触らない)
  - `ov_msckf/src/state/StateHelper.h/.cpp`: `rescale(state, s)` — p_IinG・v_IinG・全クローン位置・SLAM ランドマーク
    (表現ごと: 3D は ×s、逆深度は ÷s、方位は不変) を値と FEJ の両方で引き直し、共分散を D P D に変換
    (姿勢・バイアス・較正変数は不変)
  - `ov_msckf/src/core/VioManager.h/.cpp`: `request_scale(s)` (任意スレッドから、係数は累積)、`scale_applied()`、
    `scale_count()`。要求は次の画像更新の直前 (`apply_pending_scale`) に更新スレッドで適用し、Propagator のキャッシュを無効化
  - `ov_msckf/src/utils/ScaleUtils.h` (新規、ROS 非依存): `umeyama_scale` (相似変換の倍率の閉形式) と `ScaleEstimator`
    (対応点の蓄積、広がり (重心からの最大距離) のゲート、適用後の引き直し)
  - `ov_msckf/src/ros/ScaleCalibrator.h/.cpp` (新規、ROS1): `~scale_cmd` (Float64、係数を掛ける)、`~scale_mark` (Empty) +
    `~scale_distance` (Float64、既知距離 ÷ 推定器上の移動)、`~scale_ref_topic` (Odometry / PoseStamped /
    PoseWithCovarianceStamped、poseimu と同じ IMU 時計) との対応点から倍率を `~scale_est` (絶対の係数) に出し、
    `~scale_auto` で自動適用。状態は `~scale_status` (JSON、latch)。`~scale_inject_at`/`~scale_inject` は検査用の注入
  - `ov_msckf/src/ros/ROS1Visualizer.h/.cpp`: ScaleCalibrator を生成し `publish_state` から位置を渡す。適用後は
    pathimu の履歴も同じ比で引き直す
  - `ov_msckf/src/run_simulation.cpp`: ROS1 のとき画像ごとに `ros::spinOnce()` (参照姿勢・コマンドの配達)
  - `ov_msckf/src/test_scale.cpp` (新規、`rosrun ov_msckf test_scale`): 6 つの表現すべてで rescale の値・FEJ・共分散・往復、
    Umeyama と ScaleEstimator
  - `ov_msckf/cmake/ROS1.cmake`, `package.xml`: `topic_tools` 依存 (参照トピックの型を実行時に判別)

## 元との差分

```bash
git fetch upstream
git diff 69488123ed9362dd44b6f28e7f4680abbff1442b..ov360
```
