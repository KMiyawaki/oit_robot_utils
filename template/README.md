# ROS 2 パッケージ作成 & コマンドチートシート

新しいパッケージを作成したり、ビルド・実行したりする際によく使うコマンドのまとめです。
隣にある `*.template.*` ファイルを自パッケージにコピーして使ってください。

ファイル名からは`*.template.*`の部分を削除してください。

## パッケージの新規作成 (ros2 pkg create)

パッケージを作る際は、**「ロジック（C++/Python）」**と**「カスタムメッセージ（_msgs）」**でパッケージを分けるのが鉄則です（Humbleのビルドバグ回避のため）。

### 通常のC++パッケージを作る場合

```bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

### 純粋なPythonパッケージを作る場合

```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

### カスタムメッセージ専用パッケージを作る場合

```bash
ros2 pkg create --build-type ament_cmake my_package_msgs
```

**注意** : メッセージ専用パッケージの `package.xml` には、必ず以下の3行を綺麗に並べて書いてください（dependの罠対策）。

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## ビルドコマンド

自パッケージのルートに`build.sh`を配置し、`./build.sh`を実行してください。

## 実行とデバッグ (ros2 run / launch)

### ノードを単体で実行する

```bash
ros2 run <パッケージ名> <ノード名（実行ファイル名）>
```

### パラメータを外から上書きして実行する

```bash
ros2 run <パッケージ名> <ノード名> --ros-args -p target_param:=10.0
```

### ローンチファイルを実行する

```bash
ros2 launch <パッケージ名> <ローンチファイル名.launch.py>
```

## 便利な確認用コマンド群

トラブルシューティングや、トピックが正しく動いているか確認する時に使います。

| コマンド | 役割 |
| --- | --- |
| `ros2 node list` | 現在動いているノードの一覧を表示 |
| `ros2 topic list` | 配信中のトピック一覧を表示 |
| `ros2 topic echo /<トピック名>` | トピックの中身（データ）をリアルタイムで画面に垂れ流す |
| `ros2 topic hz /<トピック名>` | トピックが毎秒何Hzで通信できているか確認 |
| `ros2 topic info /<トピック名>` | そのトピックの「型（メッセージ名）」や通信者数を確認 |
| `ros2 interface show <メッセージ名>` | メッセージ型（例: `std_msgs/msg/String`）の内部構造を表示 |
