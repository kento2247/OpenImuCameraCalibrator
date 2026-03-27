# OpenICC

OpenICC は、動画と埋め込み IMU テレメトリを使って、カメラと IMU のキャリブレーションを行うためのツールです。主な用途は GoPro のようなアクションカメラを Structure-from-Motion、Visual-Inertial Odometry、SLAM に使える状態へ持っていくことです。

この README は、まず「OpenICC で何が取れて、何が取れないか」を先に整理します。詳細な背景や旧来の説明は [Readme.md](/mnt/mydisk/OpenImuCameraCalibrator/Readme.md) を参照してください。

## 何が取得できるか

OpenICC が直接推定できるものは次です。

| 項目 | 取得可否 | 取得方法 | 主な出力 |
|--|--|--|--|
| カメラ内部パラメータ | 可 | キャリブレーションターゲット動画から推定 | `cam/cam_calib_*.json` |
| 歪みパラメータ | 可 | カメラモデルごとに推定 | `cam/cam_calib_*.json` |
| IMU バイアス | 可 | 静止動画から平均値推定 | `imu_bias/imu_bias_*.json` |
| IMU とカメラの時間オフセット初期値 | 可 | 画像姿勢列と IMU を位置合わせ | `cam_imu/imu_to_cam_calibration_*.json` |
| IMU とカメラの回転初期値 | 可 | 同上 | `cam_imu/imu_to_cam_calibration_*.json` |
| IMU とカメラの 6DoF 外部パラメータ | 可 | 連続時間最適化で推定 | `cam_imu/cam_imu_calib_result_*.json` |
| ローリングシャッタ line delay | 条件付きで可 | 連続時間最適化で推定可能だが README 時点では実験的 | `cam_imu/cam_imu_calib_result_*.json` |
| ORB-SLAM3 向け YAML | 可 | JSON から変換スクリプトで生成 | `json2yaml.py`, `*.yaml` |

## 何がそのままは取得できないか

OpenICC だけでは直接は出ない、または別手順が必要なものは次です。

| 項目 | 取得可否 | 理由 / 代替手段 |
|--|--|--|
| IMU ノイズ密度 `NoiseGyro`, `NoiseAcc` | 直接は不可 | 通常の `run_gopro_calibration.py` では出ない。長時間静止データを作り、`fit_allan_variance` で別途推定する |
| IMU ランダムウォーク `GyroWalk`, `AccWalk` | 直接は不可 | 上と同じく Allan variance 手順が必要 |
| 温度依存パラメータ | 不可 | 温度モデルの推定機能はない |
| レンズごとの完全な汎用 ORB-SLAM3 設定 | 不可 | OpenICC は幾何・時刻・IMU 外部の推定器であり、ORB 抽出器の最適値までは決めない |
| メーカー純正の factory calibration | 不可 | OpenICC が推定するのは観測データからの実測値 |
| 磁気センサや GPS の厳密キャリブレーション | 基本不可 | GoPro テレメトリ抽出はできても、この repo の主対象は camera + accel + gyro |

## OpenICC の出力と意味

典型的な GoPro データセットでは、次の 3 本の動画を使います。

```text
MyDataset
|-- cam
|   `-- GX....MP4
|-- imu_bias
|   `-- GX....MP4
`-- cam_imu
    `-- GX....MP4
```

それぞれの役割は次です。

| フォルダ | 役割 | 主な生成物 |
|--|--|--|
| `cam` | ターゲットをゆっくり撮って内部パラメータを推定 | `cam_calib_*.json` |
| `imu_bias` | 静止状態で IMU バイアスを推定 | `imu_bias_*.json` |
| `cam_imu` | ターゲットを見ながら十分に動かし、時間オフセットと外部パラメータを推定 | `imu_to_cam_calibration_*.json`, `cam_imu_calib_result_*.json` |

主要な JSON 出力は次です。

| ファイル | 中身 |
|--|--|
| `cam/cam_calib_*.json` | `focal_length`, `principal_pt_x/y`, distortion, `image_width/height`, `fps` |
| `imu_bias/imu_bias_*.json` | `gyro_bias`, `accl_bias` |
| `cam_imu/imu_to_cam_calibration_*.json` | 初期回転と初期時間オフセット |
| `cam_imu/cam_imu_calib_result_*.json` | 最終 `q_i_c`, `t_i_c`, `time_offset_imu_to_cam_s`, line delay, trajectory |

## Hero13 でどこまで確認できたか

このリポジトリでは `data/GoPro13` を使って、少なくとも次を実確認しました。

- Hero13 MP4 から `py_gpmf_parser` で GPMF テレメトリを抽出できる
- `cam` 動画から内部パラメータを推定できる
- `imu_bias` 動画から IMU バイアスを推定できる
- `cam_imu` 動画から時間オフセット初期値と IMU-Camera 初期回転を推定できる
- 連続時間最適化で `cam_imu_calib_result_*.json` まで生成できる
- `json2yaml.py` で ORB-SLAM3 風 YAML へ変換できる

つまり Hero13 は「この repo の GoPro ワークフローに載せられる」ことを確認済みです。

## 典型的な実行フロー

### 1. 依存関係

Python 側では少なくとも次が必要です。

```bash
pip install -r requirements.txt
```

GoPro テレメトリ抽出には `py_gpmf_parser` が必要です。現在の `requirements.txt` には含まれています。

### 2. キャリブレーション

```bash
python3 python/run_gopro_calibration.py \
  --path_calib_dataset=data/GoPro13 \
  --path_to_build=build/applications \
  --checker_size_m=0.0376 \
  --image_downsample_factor=2 \
  --camera_model=FISHEYE
```

このコマンドで、内部パラメータ、IMU バイアス、時間オフセット、IMU-Camera 外部パラメータの推定まで進みます。

### 3. YAML 変換

OpenICC は最終結果を JSON で出します。ORB-SLAM3 などで使う YAML が必要なら `json2yaml.py` を使います。

```bash
python3 json2yaml.py \
  --camera_json data/GoPro13/cam/cam_calib_GX010017_fi_2.0.json \
  --imu_cam_json data/GoPro13/cam_imu/cam_imu_calib_result_GX010019.json \
  --output hero13_sample.yaml
```

## 取得不可データを埋めるには

### IMU ノイズ密度とランダムウォーク

これは通常の `run_gopro_calibration.py` では出ません。別手順で推定します。

1. カメラを完全静止したまま長時間記録する
2. テレメトリをまとめる
3. `build/applications/fit_allan_variance` を使って Allan variance を当てる
4. 得られた値を `json2yaml.py` の `--noise_gyro`, `--noise_acc`, `--gyro_walk`, `--acc_walk` に渡す

詳細は [docs/imu_noise_parameters.md](/mnt/mydisk/OpenImuCameraCalibrator/docs/imu_noise_parameters.md) を参照してください。

## どのファイルを最終的に使えばよいか

用途別に見ると次です。

| 用途 | 使うファイル |
|--|--|
| OpenICC の数値結果を確認したい | `cam/cam_calib_*.json`, `cam_imu/cam_imu_calib_result_*.json` |
| IMU バイアスだけ見たい | `imu_bias/imu_bias_*.json` |
| ORB-SLAM3 に入れたい | `json2yaml.py` で作った `*.yaml` |
| ノイズ値まで真面目に入れたい | Allan variance を別途実行して YAML を更新 |

## 関連ドキュメント

- [GoPro calibration example](/mnt/mydisk/OpenImuCameraCalibrator/docs/gopro_calibration.md)
- [IMU noise parameters](/mnt/mydisk/OpenImuCameraCalibrator/docs/imu_noise_parameters.md)
- [IMU intrinsics](/mnt/mydisk/OpenImuCameraCalibrator/docs/imu_intrinsics.md)
- [Legacy long-form README](/mnt/mydisk/OpenImuCameraCalibrator/Readme.md)
