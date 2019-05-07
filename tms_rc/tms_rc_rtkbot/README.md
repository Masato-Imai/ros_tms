# tms_rc_rtkbot

## シリアルの設定

- シリアルを一般ユーザで使う設定(初めてのときだけで良い)
```
sudo gpasswd -a ユーザ名 dialout
```
後は再ログインすればOK

- /etc/udev/rules.d/99-serial.rulesを作成し，以下の内容を記述
~~~
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", SYMLINK+="nucleo", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="04bb", ATTRS{idProduct}=="0a0e", SYMLINK+="usb_rsaq5", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="ublox", MODE="0666"
~~~

これでマイコンボードは`/dev/nucleo`として，RTK-GPSボードは`/dev/ublox`として扱える．  
(/dev/ttyACM*で数字が変わる問題に悩まされない)
(詳しくは[Qiita](https://qiita.com/caad1229/items/309be550441515e185c0)などを参考に)

## 必要なパッケージ
### aptで入手
- ros-kinetic-nmea-msgs
### GitHubから入手
- ros-kinetic-nmea-navsat-driver
### pipで入手
- pyproj