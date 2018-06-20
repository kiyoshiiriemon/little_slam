# little_slam
[友納正裕著「SLAM入門」（オーム社）](https://www.ohmsha.co.jp/book/9784274221668/)のサンプルプログラムである
[LittleSLAM](https://github.com/furo-org/LittleSLAM.git)をROSに移植したものです。

# インストール
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/kiyoshiiriemon/little_slam
cd ~/catkin_ws/
catkin_make
``` 

# 実行方法
```
rosrun little_slam little_slam_node _customize=I
```
入力は
- オドメトリ (tfの"base_link" -> "odom")
- sensor_msgs::LaserScan ("scan")

出力は
- 点群地図 sensor_msgs::PointCloud2 ("pcmap")
- ロボット軌跡 nav_msgs::Path ("path")

です。

# パラメータ
- customize
   - SLAM内部で使われている要素技術を切り替えて振る舞いの違いを観察することができます
   - AからIまであって、Iがループ解決を含むフル機能実装版です（デフォルト）
   
# 制限事項
 - 現在の実装ではロボットの中心にLidarが取り付けられているというのが前提です（座標変換が省略されています）
    - 要望があればtfから変換を読めるように直します
   
# デモ
[![](https://img.youtube.com/vi/imZT1B95MiQ/0.jpg)](https://www.youtube.com/watch?v=imZT1B95MiQ)

