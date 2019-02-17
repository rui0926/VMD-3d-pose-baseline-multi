# VMD-3d-pose-baseline-multi

このプログラムは、[VMD-Lifting](https://github.com/errno-mmd/VMD-Lifting) \(errno-mmd様\) を miu(miu200521358) がfork して、改造しました。

動作詳細等は上記URL、または [README-errno-mmd.md](README-errno-mmd.md) および [README-original.md](README-original.md) をご確認ください。

## 機能概要

以下プログラムで生成された3D関節やその他のデータから、vmd(MMDモーションデータ)ファイルを生成します

 - [miu200521358/3d-pose-baseline-vmd](https://github.com/miu200521358/3d-pose-baseline-vmd)
 - [miu200521358/3dpose_gan_vmd](https://github.com/miu200521358/3dpose_gan_vmd)
 - [miu200521358/FCRN-DepthPrediction-vmd](https://github.com/miu200521358/FCRN-DepthPrediction-vmd)
 - [miu200521358/VMD-3d-pose-baseline-multi](https://github.com/miu200521358/VMD-3d-pose-baseline-multi)


## 準備

詳細は、[Qiita](https://qiita.com/miu200521358/items/d826e9d70853728abc51)を参照して下さい。

### 依存関係

python3系 で以下をインストールして下さい

- [Tensorflow](https://www.tensorflow.org/)
- [OpenCV](http://opencv.org/)
- python-tk (Tkinter)
- PyQt5

## 実行方法

1. [Openpose簡易起動バッチ](https://github.com/miu200521358/openpose-simple) で データを解析する
1. [OpenposeTo3D.bat](OpenposeTo3D.bat) を実行する
	- [OpenposeTo3D_en.bat](OpenposeTo3D_en.bat) is in English. !! The logs remain in Japanese.
1. [miu200521358/FCRN-DepthPrediction-vmd](https://github.com/miu200521358/FCRN-DepthPrediction-vmd) で生成された深度推定データ(depth.txt)を用意する
1. [miu200521358/3d-pose-baseline-vmd](https://github.com/miu200521358/3d-pose-baseline-vmd) で生成された3D関節データ(pos.txt)、2D関節データ(smoothed.txt)を用意する
1. [miu200521358/3dpose_gan_vmd](https://github.com/miu200521358/3dpose_gan_vmd) で生成された3D関節データ(pos_gan.txt)、2D関節データ(smoothed_gan.txt)を用意する
1. [3DToVmd.bat](3DToVmd.bat) を実行する
	- [3DToVmd_en.bat](3DToVmd_en.bat) is in English. !! The logs remain in Japanese.
1. `3D解析結果ディレクトリパス` が聞かれるので、1.の結果ディレクトリパスを指定する
1. `ボーン構造CSVファイル` が聞かれるので、トレース先のモデルのボーン構造CSVファイルパスを指定する
    - センター移動の計算に身長を使用する
    - ボーン構造CSVファイルの出力方法は、[born/README.md](born/README.md)参照
    - 未指定の場合、デフォルトで[born/あにまさ式ミクボーン.csv](born/あにまさ式ミクボーン.csv)が読み込まれる
1. `足をIKで出力するか` 聞かれるので、出す場合、`yes` を入力する
    - 未指定 もしくは `yes` の場合、IKで出力する
    - `no` の場合、FKで出力する
1. `踵位置補正` が聞かれるので、踵のY軸補正値を数値(小数可)を指定する
    - マイナス値を入力すると地面に近付き、プラス値を入力すると地面から遠ざかる。
    - ある程度は自動で補正するが、ヒールのある靴や厚底などで補正しきれない場合に設定
    - 未指定の場合、「0」で補正を行わない
1. `センターZ移動倍率` が聞かれるので、センターのZ移動時の移動幅を入力する
    - 値が小さいほど、Zの移動量が少なくなり、大きいほど、移動量が増える
    - 未指定の場合、デフォルトで「5」とする
    - 「0」を指定した場合、センターZ移動を行わない
1. `円滑化度数` が聞かれるので、適当な正の整数を指定する
    - 未指定の場合、デフォルトで1回指定
1. `移動キー間引き量` が聞かれるので、移動キー(センター・IK)間引き時の移動量(小数可)を指定する
    - 指定された移動量範囲内の移動キーフレームを間引く
    - 未指定の場合、デフォルトで「0.5」とする
    - 「0」が指定された場合、間引きを行わず、以下`回転キー間引き角度`はスキップする
1. `回転キー間引き角度` が聞かれるので、回転系ボーン間引き時の回転角度(0～180度の整数のみ)を指定する
    - 回転が指定された角度範囲内であれば、キーフレームを間引く
    - 未指定の場合、デフォルトで「3」とする
1. `詳細なログを出すか` 聞かれるので、出す場合、`yes` を入力する
    - 未指定 もしくは `no` の場合、通常ログ
1. 処理開始
1. 処理が終了すると、1. の結果ディレクトリ以下に vmdファイルが出力される
	- output_{日付}_{時間}_u{直立フレームIDX}_h{踵位置補正}_z{センターZ移動倍率}_s{円滑化度数}_p{移動キー間引き量}_r{回転キー間引き角度}_full/reduce.vmd
		- キーフレームの間引きなしの場合、末尾は「full」。アリの場合、「reduce」。
	- upright.txt … 直立フレームのキー情報
1. MMDを起動し、モデルを読み込んだ後、モーションを読み込む

## ライセンス
GNU GPLv3

### 以下の行為は自由に行って下さい

- モーションの調整・改変
- ニコニコ動画やTwitter等へのモーション使用動画投稿
- モーションの不特定多数への配布
    - **必ず踊り手様や各権利者様に失礼のない形に調整してください**

### 以下の行為は必ず行って下さい。ご協力よろしくお願いいたします。

- クレジットへの記載を、テキストで検索できる形で記載お願いします。

```
ツール名：MMDモーショントレース自動化キット
権利者名：miu200521358
```
- ニコニコ動画の場合、コンテンツツリーへ [トレース自動化マイリスト](https://www.nicovideo.jp/mylist/61943776) の最新版動画を登録してください。
    - コンテンツツリーに登録していただける場合、テキストでのクレジット有無は問いません。

- モーションを配布される場合、以下文言を同梱してください。 (記載場所不問)

```
配布のモーションは、「MMDモーショントレース自動化キット」を元に作成したものです。
ご使用される際には原則として、「MMDモーショントレース自動化キット」もしくは略称の「MMD自動トレース」の使用明記と、
ニコニコ動画等、動画サイトへの投稿の場合、コンテンツツリーもしくはリンクを貼って下さい。
　登録先：MMD自動トレースマイリスト(https://www.nicovideo.jp/mylist/61943776) の最新版動画
Twitter等、SNSの場合は文言のみで構いません。
キット作者が今後の改善の参考にさせていただきます。

キット作者連絡先：
　Twitter：https://twitter.com/miu200521358
　メール：garnet200521358@gmail.com

LICENCE

MMDモーショントレース自動化キット
【Openpose】：CMU　…　https://github.com/CMU-Perceptual-Computing-Lab/openpose
【Openpose起動バッチ】：miu200521358　…　https://github.com/miu200521358/openpose-simple
【深度推定】：Iro Laina, miu200521358　…　https://github.com/miu200521358/FCRN-DepthPrediction-vmd
【Openpose→3D変換】：una-dinosauria, ArashHosseini, miu200521358　…　https://github.com/miu200521358/3d-pose-baseline-vmd
【Openpose→3D変換その2】：Dwango Media Village, miu200521358：MIT　…　https://github.com/miu200521358/3dpose_gan_vmd
【3D→VMD変換】： errno-mmd, miu200521358 　…　https://github.com/miu200521358/VMD-3d-pose-baseline-multi
```

### 以下の行為はご遠慮願います

- 自作発言
- 権利者様のご迷惑になるような行為
- 営利目的の利用
- 他者の誹謗中傷目的の利用（二次元・三次元不問）
- 過度な暴力・猥褻・恋愛・猟奇的・政治的・宗教的表現を含む（R-15相当）作品への利用
- その他、公序良俗に反する作品への利用

## 免責事項

- 自己責任でご利用ください
- ツール使用によって生じたいかなる問題に関して、作者は一切の責任を負いかねます
