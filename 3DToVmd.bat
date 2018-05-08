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


rem ---  詳細ログ有無

echo --------------
echo 詳細なログを出すか、yes か no を入力して下さい。
echo 何も入力せず、ENTERを押下した場合、通常ログのみ出力します。
set VERBOSE=2
set /P IS_DEBUG="詳細ログ[yes/no]: "

IF /I "%IS_DEBUG%" EQU "yes" (
    set VERBOSE=3
)



rem ---  X軸角度補正
rem --- set /P X_ANGLE="X軸角度補正(0～360): "
rem --- echo X_ANGLE：%X_ANGLE%

rem ---  python 実行
python applications\pos2vmd_multi.py --v %VERBOSE% --t %TARGET_DIR% --a 0

