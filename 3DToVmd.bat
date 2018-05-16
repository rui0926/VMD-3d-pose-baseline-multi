@echo off
rem --- 
rem ---  3D の 関節データから vmdデータに変換
rem --- 

rem ---  カレントディレクトリを実行先に変更
cd /d %~dp0

rem ---  解析結果JSONディレクトリパス
echo 3d-pose-baseline-vmdの解析結果ディレクトリのフルパスを入力して下さい。(3d_{実行日時}_idx00)
echo この設定は半角英数字のみ設定可能で、必須項目です。
set TARGET_DIR=
set /P TARGET_DIR=■3D解析結果ディレクトリパス: 
rem echo TARGET_DIR：%TARGET_DIR%

IF /I "%TARGET_DIR%" EQU "" (
    ECHO 3D解析結果ディレクトリパスが設定されていないため、処理を中断します。
    EXIT /B
)

rem ---  グローバルX軸角度補正

echo --------------
set GROBAL_X_ANGLE=16
echo 3D化した際にグローバルX軸に若干傾くのを補正します
echo 0～360度の数字のみを入力して下さい。
echo 何も入力せず、ENTERを押下した場合、%GROBAL_X_ANGLE%度回転します。
set /P GROBAL_X_ANGLE="グローバルX軸角度補正: "

rem ---  同軸フレーム間引き角度

echo --------------
set SAME_DECIMATION_ANGLE=30
echo フレームの間引きに使用する角度を指定します
echo 同じ軸(X軸方向への回転が続く場合等)に指定された角度以内の回転があった場合に間引きされます。
echo 0～360度の数字のみを入力して下さい。
echo 何も入力せず、ENTERを押下した場合、%SAME_DECIMATION_ANGLE%度間引きます。
set /P SAME_DECIMATION_ANGLE="同軸フレーム間引き角度: "

rem ---  異軸フレーム間引き角度

echo --------------
set DIFF_DECIMATION_ANGLE=6
echo フレームの間引きに使用する角度を指定します
echo 違うじ軸(X軸方向への回転からZ軸方向に回転した場合等)に指定された角度以内の回転があった場合に間引きされます。
echo 0～360度の数字のみを入力して下さい。
echo 何も入力せず、ENTERを押下した場合、%DIFF_DECIMATION_ANGLE%度間引きます。
set /P DIFF_DECIMATION_ANGLE="異軸フレーム間引き角度: "


rem ---  詳細ログ有無

echo --------------
echo 詳細なログを出すか、yes か no を入力して下さい。
echo 何も入力せず、ENTERを押下した場合、通常ログのみ出力します。
set VERBOSE=2
set /P IS_DEBUG="詳細ログ[yes/no]: "

IF /I "%IS_DEBUG%" EQU "yes" (
    set VERBOSE=3
)


rem ---  python 実行
python applications\pos2vmd_multi.py -v %VERBOSE% -t %TARGET_DIR% -x %GROBAL_X_ANGLE% -s %SAME_DECIMATION_ANGLE% -d %DIFF_DECIMATION_ANGLE%

