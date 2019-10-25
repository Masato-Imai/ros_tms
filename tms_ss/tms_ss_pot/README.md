# 4台のLRFによるトラッキング

## 0. Precondition
! 設定済ならこの項はスキップ
### 0.0. Set IP Address
IPアドレスは以下のように手動で設定されていると仮定

| Name | IP Address |
|:-----:|:-----:|
| Router | 192.168.12.1 |
| Server(ROS_MASTER_URI) | 192.168.12.2 |
| P2Sen_1 | 192.168.12.11 |
| P2Sen_2 | 192.168.12.12 |
| P2Sen_3 | 192.168.12.13 |
| P2Sen_4 | 192.168.12.14 |

この設定でROSの通信が正常に行える状態であることを確認

また，ここではroslaunchを行うPCとServerが同一であるとする

remoteのlaunchを行う際は，roslaunchを行うPCのROS_MASTER_URIがコピーされるので，`~/.bashrc`の設定に注意

### 0.1. Configure Network 
roslaunchを行うPCで`/etc/hosts`を編集
```bash
sudo nano /etc/hosts
```

```bash
127.0.0.1	localhost
127.0.1.1	CFSZ6-4L

# For ROS Connection   ## 追記分
192.168.12.11 p2sen_1
192.168.12.12 p2sen_2
192.168.12.13 p2sen_3
192.168.12.14 p2sen_4
#

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
```

roslaunchを行うPCで`~/.bashrc`に以下を追記
```bash
export ROSLAUNCH_SSH_UNKNOWN=1
```

### 0.2. SSH
roslaunchを行うPCにおいて，`~/.ssh/known_hosts`の記録を消去した上で，
```bash
ssh odroid@p2sen_* -oHostKeyAlgorithms='ssh-rsa'
```
と手でログインし，RSAアルゴリズムを使うよう指定しておく

### 0.3. Create `.bash` File in Odroid
launchファイルで指定した，`/home/odroid/env.bash`というスクリプトをodroidのホームディレクトリに作成

(remoteの際，これが`~/.bashrc`の代わりになる?)
```bash
#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=192.168.12.1* # (*: 1 ~ 4のそれぞれの数)
exec "$@"

```

実行権限の付与を忘れずに

## 1. Bring Up P2-Sen
```bash
roslaunch tms_ss_pot bringup_multiple_poles.launch
```

上記のコマンドで問題がある場合は､各Odroidへsshし，

```bash
roslaunch tms_ss_pot urg*.launch  # アスタリスクは1~4の番号
```

## 2. Run Tracker
```bash
roslaunch tms_ss_pot p2sen_tracker.launch
```

このとき，tms_ss_pot配下にある`lrf*.yaml`に注意 \
このファイルは背景差分なので，設置位置などが変わった場合はこのファイルを消してから立ち上げる．

## Ex1. Set Parameter
LRFの設置位置や移動体を追跡する範囲を設定する際は,
`config.yaml`内の値を変更.

## Ex2. Laser Calibration
LRFの設置位置を調整するときは,
```bash
roslaunch tms_ss_pot laser_calib.launch
```
で,LaserのMarkerArrayを表示して調整.
