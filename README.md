# ros2_safety_monitor

本パッケージは，ロボットの二次元位置 `(x, y)` を入力として受け取り，  
原点を中心とした同心円状の安全ゾーンとの距離に基づいて  
`safe` / `warning` / `danger` の三段階で安全状態を判定する ROS 2 ノードである。

実ロボットや実センサを使用せず，数学的モデルのみで安全監視アルゴリズムの検証が可能であり，  
教育用途・デバッグ用途・シミュレーション用途において有用なパッケージである。

---

## パッケージの目的

ロボットの運用環境においては，走行可能領域・注意領域・立入禁止領域といった  
安全ゾーンの概念 が重要である。  

しかし実機を用いた検証はコストが高く，事前に安全判定ロジックを確立したい場合が多い。

本パッケージは次のような目的で設計されている：

- 安全判定ロジックを数学的に検証できること
- 位置座標のみで危険状態を判定できる最小限のモデルを提供すること
- ROS 2 の通信モデル（購読 / 公開）の理解を深めること
- テスト容易性を考慮したモジュール構成を示すこと
- GitHub Actions を用いた品質保証手法を示すこと

---

## ノード構成

本パッケージには 1 つのノードが存在する。

## safety_monitor ノード

| 役割 | 説明 |
|------|------|
| 入力 | `/robot_position (geometry_msgs/msg/Point)` |
| 出力 | `/safety_status (std_msgs/msg/String)` |
| 機能 | ロボット位置を受け取り，安全状態を判定して出力する |

ノード内部には ROS 非依存のロジック部分を分離し，  
テストや保守性の向上を図っている。

---

## 使用トピック

### 購読トピック

### `/robot_position`
- 型: `geometry_msgs/msg/Point`
- 説明:
  ロボットの現在位置を表す。`x` と `y` のみを利用し，`z` は使用しない。

---

## パブリッシュトピック

### `/safety_status`
- 型: `std_msgs/msg/String`
- 説明: 
  安全状態を文字列として出力する。取りうる値は次の 3 種類である。

| 状態 | 説明 |
|------|------|
| `"safe"` | 中心付近の安全領域にある |
| `"warning"` | 危険領域の境界に近づいている |
| `"danger"` | 危険領域に入った，または大きく逸脱している |

---

## 安全判定ロジック

安全状態は，原点 `(0, 0)` からのユークリッド距離によって決定する。

\[
d = \sqrt{x^2 + y^2}
\]

本パッケージでは次のように閾値を設定している。

| 距離 d | 状態 |
|--------|-------|
| `d < 6.0` | `"safe"` |
| `6.0 <= d < 10.0` | `"warning"` |
| `d >= 10.0` | `"danger"` |

### なぜこの設計なのか？
- 安全領域 (safe):  
  ロボットの通常動作範囲を想定しており，特に制限を設けない。

- 注意領域 (warning):  
  境界付近における速度制御・作業者への通知などの判断材料となる。

- 危険領域 (danger):  
  停止や緊急動作を想定する領域であり，明確に区別する必要がある。

###  実装上の特徴
- 判定ロジックを `safety_logic.py` として分離  
- ROS 依存を排除し，pytest による単体テストが可能  
- 学術的・教育的に扱いやすい構造である  

---

## 実行手順
本パッケージを正しく動作させるためには，3つのターミナルを使用することを推奨する。
以下に，ROS 2 初心者でも確実に動かせる手順を示す。

---

###  Terminal 1：Safety Monitor ノードの起動

1. ターミナルを開く
2. ワークスペースを読み込む：

```bash
source ~/ros2_ws/install/setup.bash
```

3. ノードの起動
```ros2 run ros2_safety_monitor safety_node```

起動すると次のようなログが表示される：
```[INFO] [safety_monitor]: Safety Monitor Node started.```

### Terminal 2：ロボット位置情報の送信（テスト用）

このターミナルでは `/robot_position` に位置データを送信して動作確認を行う。

```source ~/your_ws/install/setup.bash```


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

### Terminal 3：Safety Status の確認

Safety Monitor が判定した結果は `/safety_status` に文字列として出力される。
```
source ~/your_ws/install/setup.bash
ros2 topic echo /safety_status
```

出力例：

```
data: "safe"
---
data: "warning"
---
data: "danger"
```

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

## インストール方法

ROS 2 パッケージの標準的な手順に従う：

```
cd ~/your_ws/src
git clone https://github.com/Daiki606/safety_monitor_ros2.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## ライセンス

本ソフトウェアは MIT License に基づいて公開する。
詳細は `LICENSE` を参照のこと。


## 著作権表示
© 2025 Daiki Yamashita
