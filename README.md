![GitHub last commit](https://img.shields.io/github/last-commit/yuhi-sa/odrive_ros)
![GitHub commit activity](https://img.shields.io/github/commit-activity/m/yuhi-sa/odrive_ros)
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/yuhi-sa/odrive_ros)
![GitHub top language](https://img.shields.io/github/languages/top/yuhi-sa/odrive_ros)
# odrive_ros
[ODrive motor](https://odriverobotics.com/)を動かすROSパッケージ．  
位置制御と速度制御の切り替え，指令値の送信． 位置と速度のリアルタイムプロット．

# 環境
- [Ubuntu 16.04 LTS](https://wiki.ubuntu.com/XenialXerus/ReleaseNotes/Ja#Ubuntu_16.04.2BMG4wwDCmMPMw7TD8MMk-)
- [ROS kinetic](http://wiki.ros.org/ja/kinetic/Installation/Ubuntu)
- [odrive 0.4.12](https://pypi.org/project/odrive/)

# 使い方
1. [ROSワークスペースの作成](http://wiki.ros.org/ja/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```
2. [odrive_rosリポジトリのクローン](https://git-scm.com/book/ja/v2/Git-%E3%81%AE%E5%9F%BA%E6%9C%AC-Git-%E3%83%AA%E3%83%9D%E3%82%B8%E3%83%88%E3%83%AA%E3%81%AE%E5%8F%96%E5%BE%97)
```
cd ~/catkin_ws/src
git clone clone URL
```
3. 起動
```
cd ~/catkin_ws
catkin_make #catkinパッケージをビルド
roscore 

#別のターミナルを開く
rosrun odrive_ros controller.py
#起動と同時にODriveの初期設定をする．(1分ぐらい待つ)
#太郎準備完了ってでたらOK

#別のターミナルを開く
rosrun odrive_ros command.py
#ここで指令値の入力および，制御方法の切り替えを行う
```
4. コマンド
- vel：速度制御に切替
- pos：位置制御に切替
- stop：停止

5. GUI操作
```
#新しいターミナルを開く
roslaunch rosbridge_server rosbridge_websocket.launch
```
詳しくは，[odrive_gui](https://github.com/yuhi-sa/odrive_gui)

6. パッケージの概要
<img width="1060" alt="fig1" src="https://user-images.githubusercontent.com/62089243/126146584-b28f1759-a4d6-49a5-b151-2d210b9e83d0.png">

7. ROSでのデータの可視化
- rqt_plot：x軸が時間，y軸がトピックの値の2次元プロット
```
#別のターミナルを開く
rosrun rqt_plot rqt_plot
```

- rqt_graph：現在動作しているノードの情報を表示
```
#別のターミナルを開く
rosrun rqt_graph rqt_graph
```

# 参考
- [ODrive Documentation](https://docs.odriverobotics.com/)
- [neomanic/odrive_ros](https://github.com/neomanic/odrive_ros)
- [Raspberry Pi Mouse Simulator's Tutorial](https://raspimouse-sim-tutorial.gitbook.io/project/)
- [ROS Melodic Moreniaを使ってPub/Subでサーボモータを動かす](https://tkrel.com/9301)
- [ROS javascriptでメッセージを配信(rosbridge + rosjs)](https://symfoware.blog.fc2.com/blog-entry-2292.html)

