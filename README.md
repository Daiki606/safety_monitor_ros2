# ros2_safety_monitor

ロボットの2次元位置 `(x, y)` を入力として受け取り，原点を中心とした同心円状の安全ゾーンに基づいて  
`safe` / `warning` / `danger` のいずれかの安全状態を判定し，トピックとして公開する ROS 2 パッケージである。

本パッケージは、実ロボットやセンサを必要とせず、数学的な計算のみで安全監視ロジックをテスト・検証できることを目的としている。

---

## 機能概要

本ノードは以下の処理を行う：

1. `/robot_position` からロボットの現在位置 (Point 型) を購読する  
2. 原点 (0, 0) からの距離を計算する  
3. 距離に応じて安全状態を判定する  
4. 判定結果 (`safe`, `warning`, `danger`) を `/safety_status` にパブリッシュする  

---

## 安全ゾーンの定義

本パッケージでは以下の3段階のゾーンを定義している。

| 距離 | 状態 |
|------|------|
| **0.0 m 〜 < 6.0 m** | 🟢 `safe` |
| **6.0 m 〜 < 10.0 m** | 🟡 `warning` |
| **10.0 m 〜** | 🔴 `danger` |

距離の計算はユークリッド距離である。

```python
distance = sqrt(x*x + y*y)
```

## 使用トピック

購読トピック

```/robot_position```

型: ```geometry_msgs/msg/Point```

説明: ロボットの現在位置を表す。x, y 座標を使用し、z は無視する。

##パブリッシュトピック

```/safety_status```

型: `std_msgs/msg/String`

説明: 安全状態を文字列で通知する

-`"safe"`
-`"warning"`
-`"danger"`

## 判定ロジック

判定処理は `ros2_safety_monitor/safety_logic.py` に実装されている。
```
if distance < 6.0:
    return "safe"
elif 6.0 <= distance < 10.0:
    return "warning"
else:
    return "danger"
```
ロジックを独立させることで、ROS 2 を使わずに pytest によるテストが可能となっている。


## ノードの起動方法
1. ビルド
```
   cd ~/ros2_ws/src
git clone https://github.com/Daiki606/safety_monitor_ros2.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

2. ノードの起動
```ros2 run ros2_safety_monitor safety_node```

## 動作確認例

例1：安全ゾーン内

```ros2 topic pub /robot_position geometry_msgs/msg/Point "{x: 2.0, y: 3.0, z: 0.0}"```

`/safety_status 出力:`
```data: "safe"```

例2：warning ゾーン

```ros2 topic pub /robot_position geometry_msgs/msg/Point "{x: 7.0, y: 1.0, z: 0.0}"```

出力:

```data: "warning"```

例3：danger ゾーン

```ros2 topic pub /robot_position geometry_msgs/msg/Point "{x: 10.5, y: 0.0, z: 0.0}"```

出力:

```data: "danger"```

## テストについて (pytest)
ロジック部分は ROS 2 非依存のため pytest でテストできる。

ローカル実行
`pytest`

テスト内容の例:
-`safe` / `warning` / `danger` の判定
-6.0 m, 10.0 m の境界条件テスト

GitHub Actions 上でも自動的に:
-`flake8`
-`pytest`
が実行されるように設定してある。


## ディレクトリ構成
```
safety_monitor_ros2/
 ├── ros2_safety_monitor/
 │     ├── __init__.py
 │     ├── safety_logic.py
 │     └── safety_node.py
 ├── test/
 │     └── test_safety_logic.py
 ├── resource/
 │     └── ros2_safety_monitor
 ├── .github/
 │     └── workflows/
 │           └── test.yml
 ├── package.xml
 ├── setup.py
 ├── setup.cfg
 ├── LICENSE
 └── README.md
```

## ライセンス

本ソフトウェアは MIT License に基づいて公開する。
詳細は `LICENSE` を参照のこと。


## 著作権表示
© 2025 Daiki Yamashita
