# VMD-3d-pose-baseline-multi

このプログラムは、[VMD-Lifting](https://github.com/errno-mmd/VMD-Lifting) \(errno-mmd様\) を miu(miu200521358) がfork して、改造しました。

動作詳細等は上記URL、または [README-errno-mmd.md](README-errno-mmd.md) および [README-original.md](README-original.md) をご確認ください。

## 機能概要

- [miu200521358/VMD-3d-pose-baseline-multi](https://github.com/miu200521358/VMD-3d-pose-baseline-multi) で生成された3D関節データから、vmd(MMDモーションデータ)ファイルを生成します

## 準備

詳細は、[ブロマガ]()を参照して下さい。

### 依存関係

python3系 で以下をインストールして下さい

- [Tensorflow](https://www.tensorflow.org/)
- [OpenCV](http://opencv.org/)
- python-tk (Tkinter)
- PyQt5

## 実行方法

1. [VMD-3d-pose-baseline-multi](https://github.com/miu200521358/VMD-3d-pose-baseline-multi) で生成された3D関節データを生成する
1. [3DToVmd.bat](3DToVmd.bat) を実行する
1. `3Dデータの出力結果ディレクトリパス` が聞かれるので、1.の結果ディレクトリパスを指定する
1. 処理開始
1. 処理が終了すると、1. の結果ディレクトリ以下に vmdファイルが出力される
1. MMDを起動し、モデルを読み込んだ後、モーションを読み込む

## ライセンス
GNU GPLv3

以下の行為は自由に行って下さい

- モーションの調整・改変
- このツールを使用したモーションの不特定多数への配布
    - ただし、必ず踊り手様や権利者様に失礼のない形に調整してください

以下の行為は必ず行って下さい。ご協力よろしくお願いいたします。

- クレジットへの記載（記載場所は不問）
- コンテンツツリーへの動画(sm33161300)登録
- twitter等、SNSへの投稿はOKですが、クレジットとして `miu200521358` を入れて下さい

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
