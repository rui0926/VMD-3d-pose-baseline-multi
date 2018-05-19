# VMD-3d-pose-baseline-multi

このプログラムは、[VMD-Lifting](https://github.com/errno-mmd/VMD-Lifting) \(errno-mmd様\) を miu(miu200521358) がfork して、改造しました。

動作詳細等は上記URL、または [README-errno-mmd.md](README-errno-mmd.md) および [README-original.md](README-original.md) をご確認ください。

## 機能概要

- [miu200521358/VMD-3d-pose-baseline-multi](https://github.com/miu200521358/VMD-3d-pose-baseline-multi) で生成された3D関節データから、vmd(MMDモーションデータ)ファイルを生成します

## 準備

詳細は、[Qiita](https://qiita.com/miu200521358/items/d826e9d70853728abc51)を参照して下さい。

### 依存関係

python3系 で以下をインストールして下さい

- [Tensorflow](https://www.tensorflow.org/)
- [OpenCV](http://opencv.org/)
- python-tk (Tkinter)
- PyQt5

## 実行方法

1. [VMD-3d-pose-baseline-multi](https://github.com/miu200521358/VMD-3d-pose-baseline-multi) で生成された3D関節データを用意する
1. [3DToVmd.bat](3DToVmd.bat) を実行する
1. `3D解析結果ディレクトリパス` が聞かれるので、1.の結果ディレクトリパスを指定する
1. `ボーン構造CSVファイル` が聞かれるので、トレース先のモデルのボーン構造CSVファイルパスを指定する
    - センター移動の計算に身長を使用する
    - ボーン構造CSVファイルの出力方法は、[born/README.md](born/README.md)参照
    - 未指定の場合、デフォルトで[born/あにまさ式ミクボーン.csv](born/あにまさ式ミクボーン.csv)が読み込まれる
1. `直立姿勢フレーム番号` が聞かれるので、できるだけ人物が正面向きで直立(首から足まで)しているフレームのINDEX(1始まり)を指定する
    - 指定されたフレームの人物の位置がMMDのセンター中央位置となる
    - 未指定の場合、デフォルトで1フレーム目を直立として扱う
1. `センターXY移動倍率` が聞かれるので、センターのXY移動時の移動幅を入力する
    - 値が小さいほど、XYの移動量が少なくなり、大きいほど、移動量が増える
    - 未指定の場合、デフォルトで「30」とする
1. `センターZ移動倍率` が聞かれるので、センターのZ移動時の移動幅を入力する
    - 値が小さいほど、Zの移動量が少なくなり、大きいほど、移動量が増える
    - 未指定の場合、デフォルトで「10」とする
1. `グローバルX軸補正角度` が聞かれるので、適当な角度を指定する
    - 未指定の場合、デフォルトで17度指定
    - モーションにより、微妙に異なる場合があるので、適宜調整
1. `同軸フレーム間引き角度` が聞かれるが、これは現在未使用のパラメータなので無視
1. `異軸フレーム間引き角度` が聞かれるが、これは現在未使用のパラメータなので無視
1. `詳細なログを出すか` 聞かれるので、出す場合、`yes` を入力する
    - 未指定 もしくは `no` の場合、通常ログ
1. 処理開始
1. 処理が終了すると、1. の結果ディレクトリ以下に vmdファイルが出力される
1. MMDを起動し、モデルを読み込んだ後、モーションを読み込む

## ライセンス
GNU GPLv3

以下の行為は自由に行って下さい

- モーションの調整・改変
- ニコニコ動画やTwitter等へのモーション投稿
- このツールを使用したモーションの不特定多数への配布
    - ただし、必ず踊り手様や権利者様に失礼のない形に調整してください

以下の行為は必ず行って下さい。ご協力よろしくお願いいたします。

- クレジットへの記載
    - 記載場所は不問。名前は`miu200521358`でお願いします。
- ニコニコ動画の場合、コンテンツツリーへの動画\([sm33217872](http://www.nicovideo.jp/watch/sm33217872)\)登録
- モーションを配布する場合、以下を同梱してください (記載場所不問)

```
LICENCE

モーショントレース自動化キット
【Openpose】：CMU　…　https://github.com/CMU-Perceptual-Computing-Lab/openpose
【起動バッチ】：miu200521358　…　https://github.com/miu200521358/openpose-simple
【Openpose→3D変換】：una-dinosauria, ArashHosseini, miu200521358　…　https://github.com/miu200521358/3d-pose-baseline-vmd
【3D→VMD変換】： errno-mmd, miu200521358 　…　https://github.com/miu200521358/VMD-3d-pose-baseline-multi
```

以下の行為はご遠慮願います

- 自作発言
- 権利者様のご迷惑になるような行為
- 営利目的の利用
- 他者の誹謗中傷目的の利用（二次元・三次元不問）
- 過度な暴力・猥褻・恋愛・猟奇的・政治的・宗教的表現を含む（R-15相当）作品への利用
- その他、公序良俗に反する作品への利用

## 免責事項

- 自己責任でご利用ください
- ツール使用によって生じたいかなる問題に関して、作者は一切の責任を負いかねます
