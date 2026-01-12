# ros2_safety_monitor

ロボットの二次元位置 `(x, y)` を入力として受け取り，
原点からの距離に基づいて安全状態を判定する ROS 2 ノードである。

---

## ノード概要

### safety_monitor ノード

- **購読トピック**  
  `/robot_position` (`geometry_msgs/msg/Point`)

- **パブリッシュトピック**  
  `/safety_status` (`std_msgs/msg/String`)

---

## 動作仕様

ロボット位置 `(x, y)` から次の基準で状態を判定する。

| 距離 d | 出力 |
|------|------|
| `d < 6.0` | `"safe"` |
| `6.0 <= d < 10.0` | `"warning"` |
| `d >= 10.0` | `"danger"` |

---

## 実行方法

### ノード起動

```bash
source ~/your_ws/install/setup.bash
ros2 run ros2_safety_monitor safety_node
```

※ `your_ws` は任意の ROS 2 ワークスペース名を表す。

---

## 動作確認例
```bash
ros2 topic pub /robot_position geometry_msgs/msg/Point "{x: 2.0, y: 3.0, z: 0.0}"
```

---

### 出力例：
```
data: "safe"
```

---

## テスト
本パッケージにはシェルスクリプトによる簡単なテストを含む。
```bash
bash test_node.sh
```
 ※ 本テストは ROS 2 がインストール済みの環境での実行を想定している。

 ---

 ## ライセンス
 MIT License
 
SPDX-License-Identifier: MIT

