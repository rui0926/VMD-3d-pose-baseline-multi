@echo off
rem --- 
rem ---  3D の 関節データから vmdデータに変換
rem --- 

rem ---  カレントディレクトリを実行先に変更
cd /d %~dp0

rem ---  3Dディレクトリ確認
set /P TARGET_DIR="3Dデータの出力結果ディレクトリパス: "
rem --- echo PERSON_IDX：%TARGET_DIR%

rem ---  X軸角度補正
rem --- set /P X_ANGLE="X軸角度補正(0～360): "
rem --- echo X_ANGLE：%X_ANGLE%

rem ---  python 実行
rem --- 詳細なログが不要な場合、--v の後ろの数字を「2」に設定して下さい
rem --- 詳細なログが必要な場合、--v の後ろの数字を「3」に設定して下さい
python applications\pos2vmd_multi.py --v 2 --t %TARGET_DIR% --a 0

