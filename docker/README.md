# Docker 開発環境

ADI IMU TR Driver ROS2 (Robot Operating System 2) の
開発用 Docker 環境の使い方をまとめる。
ROS2 のビルド・実行・テスト環境をコンテナに
閉じ込めて、ホストを汚さずに開発を回すための環境。

## 構成ファイル

表 1 `docker/` 配下のファイル一覧

| ファイル | 役割 |
|---|---|
| `Dockerfile.humble` | ROS2 Humble (Ubuntu 22.04) 用イメージ定義 |
| `Dockerfile.jazzy` | ROS2 Jazzy (Ubuntu 24.04) 用イメージ定義 |
| `docker-compose.yml` | `humble` / `jazzy` 2 サービスの起動定義 |
| `.env` | UID/GID やデバイスパスの環境変数（git 管理外） |

Humble 版と Jazzy 版は別イメージとしてビルドされ、
`docker-compose.yml` 内の `humble` / `jazzy` という
2 つのサービスとして切り替えて使う。

## 前提

- Docker および Docker Compose v2 がインストール済み
- IMU (Inertial Measurement Unit) 実機を使う場合は
  USB (Universal Serial Bus) シリアル接続されていること
- GUI (RViz 等) を使う場合はホストが X11 環境であること

## .env の設定

`.env` は git 管理外（`.gitignore` 対象）なので、
クローン直後は自分で用意する必要がある。
以下を `docker/.env` として作成する。

```dotenv
DEV_UID=1000
DEV_GID=1000
DEV_USER=hogehoge
VIDEO_GID=44
RENDER_GID=110
# IMU 実機を使うときだけ有効化する。
# 未設定なら /dev/null にフォールバックして起動できる。
# IMU_DEVICE=/dev/ttyACM0
```

表 2 `.env` の変数一覧

| 変数 | 既定値 | 説明 |
|---|---|---|
| `DEV_UID` | 1000 | コンテナ内ユーザーの UID |
| `DEV_GID` | 1000 | コンテナ内ユーザーの GID |
| `DEV_USER` | hogehoge | コンテナ内ユーザー名 |
| `VIDEO_GID` | 44 | GPU アクセス用 video グループ GID |
| `RENDER_GID` | 110 | GPU アクセス用 render グループ GID |
| `IMU_DEVICE` | （未設定） | IMU 実機のデバイスパス |

`DEV_UID` / `DEV_GID` はホストの自分の値に合わせると、
マウントしたワークスペースの成果物（`build/`,
`install/` 等）の所有者がホスト側と一致して扱いやすい。
ホストでの確認方法は次の通り。

```bash
id -u   # → DEV_UID に設定
id -g   # → DEV_GID に設定
```

`VIDEO_GID` / `RENDER_GID` は GPU を使う場合に
ホストの実際のグループ GID へ合わせる。

```bash
getent group video    # → VIDEO_GID
getent group render   # → RENDER_GID
```

## ビルド

コンテナイメージを作成する。Humble と Jazzy で別々。

```bash
# docker/ ディレクトリで実行する想定
cd docker

# Humble 版イメージをビルド
docker compose build humble

# Jazzy 版イメージをビルド
docker compose build jazzy
```

ROS 依存パッケージは `rosdep` でビルド時に焼き込むので、
コンテナ起動のたびに依存をインストールする必要はない。

## 起動とコンテナ内操作

```bash
cd docker

# Jazzy 版を起動（バックグラウンド）
docker compose up -d jazzy

# コンテナ内 bash に入る
docker exec -it adi-imu-tr-driver-jazzy bash

# 停止・削除
docker compose down
```

Humble 版を使う場合はサービス名・コンテナ名を
`humble` / `adi-imu-tr-driver-humble` に置き換える。

表 3 サービスとコンテナ名の対応

| サービス | イメージ | コンテナ名 |
|---|---|---|
| `humble` | `adi-imu-tr-driver:humble` | `adi-imu-tr-driver-humble` |
| `jazzy` | `adi-imu-tr-driver:jazzy` | `adi-imu-tr-driver-jazzy` |

ワークスペースはコンテナ内の
`~/ros2_ws/src/adi_imu_tr_driver_ros2` に
read-write でマウントされ、`colcon build` の成果物は
ホスト側にそのまま残る。

## コンテナ内でのビルド・実行

```bash
# 作業ディレクトリは ~/ros2_ws（.bashrc で設定済み）
cd ~/ros2_ws

# パッケージをビルド
colcon build --packages-select adi_imu_tr_driver_ros2
source install/setup.bash

# 既定設定で起動（Attitude モード, 100Hz, RViz あり）
ros2 launch adi_imu_tr_driver_ros2 adis_rcv_csv.launch.py
```

`source /opt/ros/<distro>/setup.bash` と
`install/setup.bash` の読み込みは `.bashrc` に
仕込んであるので、コンテナに入れば自動で有効になる。

## IMU 実機の接続

`docker-compose.yml` は `privileged` を使わず、
USB シリアルデバイスだけを明示的にマッピングする。
実機を使うときは `.env` で `IMU_DEVICE` を指定する。

```dotenv
IMU_DEVICE=/dev/ttyACM0
```

ホスト側のデバイスパスを確認する。

```bash
ls -l /dev/ttyACM*
```

`IMU_DEVICE` を設定して起動すると、ホストの実機が
コンテナ内では常に `/dev/ttyACM0` として見える。
未設定の場合は `/dev/null` にフォールバックするので、
実機が無くてもコンテナ自体は起動できる。

## GUI (RViz) を使う

`network_mode: host` と X11 ソケットのマウントにより、
コンテナ内の RViz をホストの画面に表示できる。
起動前にホスト側で X11 アクセスを許可する。

```bash
xhost +local:docker
```

`DISPLAY` 環境変数は `docker-compose.yml` が
ホストから引き継ぐので、追加設定は不要。

## マウント内容

表 4 ホストからコンテナへのマウント一覧

| ホスト側 | コンテナ側 | 用途 |
|---|---|---|
| `..`（リポジトリ） | `~/ros2_ws/src/adi_imu_tr_driver_ros2` | ソース・成果物 |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 GUI 表示 |

## トラブルシューティング

表 5 よくある問題と対処

| 症状 | 対処 |
|---|---|
| 成果物の所有者が root になる | `.env` の `DEV_UID`/`DEV_GID` をホストの `id` に合わせて再ビルド |
| RViz が表示されない | ホストで `xhost +local:docker` を実行 |
| IMU が見つからない | `.env` の `IMU_DEVICE` を実機のパスに設定して再起動 |
| GPU が使えない | `getent group video/render` の GID を `.env` に反映 |
