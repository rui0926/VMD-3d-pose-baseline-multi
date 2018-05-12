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

rem ---  上半身グローバルX軸角度補正

echo --------------
set GROBAL_UPPER_X_ANGLE=37
echo 3D化した際に上半身がグローバルX軸に若干傾くのを補正します
echo 0～360度の数字のみを入力して下さい。
echo 何も入力せず、ENTERを押下した場合、%GROBAL_UPPER_X_ANGLE%度回転します。
set /P GROBAL_UPPER_X_ANGLE="上半身グローバルX軸角度補正: "

rem ---  下半身グローバルX軸角度補正

echo --------------
set GROBAL_LOWER_X_ANGLE=17
echo 3D化した際に下半身がグローバルX軸に若干傾くのを補正します
echo 0～360度の数字のみを入力して下さい。
echo 何も入力せず、ENTERを押下した場合、%GROBAL_LOWER_X_ANGLE%度回転します。
set /P GROBAL_LOWER_X_ANGLE="上半身グローバルX軸角度補正: "


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
python applications\pos2vmd_multi.py --v %VERBOSE% --t %TARGET_DIR% --u %GROBAL_UPPER_X_ANGLE% --l %GROBAL_LOWER_X_ANGLE%

