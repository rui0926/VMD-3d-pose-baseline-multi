#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pos2vmd.py - convert joint position data to VMD

from __future__ import print_function

def usage(prog):
    print('usage: ' + prog + ' POSITION_FILE VMD_FILE')
    sys.exit()

import re
from PyQt5.QtGui import QQuaternion, QVector4D, QVector3D, QVector2D, QMatrix4x4
from VmdWriter import VmdBoneFrame, VmdInfoIk, VmdShowIkFrame, VmdWriter
import argparse
import logging
import datetime
import numpy as np
import csv
from decimal import Decimal

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ディクショナリ型で各ボーンごとのキーフレームリストを作成する
bone_frame_dic = {
    "上半身":[],
    "下半身":[],
    "首":[],
    "頭":[],
    "左肩":[],
    "左腕":[],
    "左ひじ":[],
    "右肩":[],
    "右腕":[],
    "右ひじ":[],
    "左足":[],
    "左ひざ":[],
    "右足":[],
    "右ひざ":[],
    "センター":[],
    "グルーブ":[],
    "左足ＩＫ":[],
    "右足ＩＫ":[]
}

def positions_to_frames(pos, frame=0, xangle=0):
    logger.debug("角度計算 frame={0}".format(str(frame)))

	# 補正角度のクォータニオン
    decrease_correctqq = QQuaternion.fromEulerAngles(QVector3D(xangle * 0.5, 0, 0))
    increase_correctqq = QQuaternion.fromEulerAngles(QVector3D(xangle * 2, 0, 0))
    normal_correctqq = QQuaternion.fromEulerAngles(QVector3D(xangle, 0, 0))

    """convert positions to bone frames"""
    # 上半身
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8f\xe3\x94\xbc\x90\x67' # '上半身'
    direction = pos[8] - pos[7]
    up = QVector3D.crossProduct(direction, (pos[14] - pos[11])).normalized()
    upper_body_orientation = QQuaternion.fromDirection(direction, up)
    initial = QQuaternion.fromDirection(QVector3D(0, 1, 0), QVector3D(0, 0, 1))
    upper_body_rotation = upper_body_orientation * initial.inverted()

    # 補正をかけて回転する
    if upper_body_rotation.toEulerAngles().y() < 30 and upper_body_rotation.toEulerAngles().y() > -30:
        # 前向きは増量補正
        upper_body_rotation = upper_body_rotation * increase_correctqq
    elif upper_body_rotation.toEulerAngles().y() < -120 or upper_body_rotation.toEulerAngles().y() > 120:
        # 後ろ向きは通常補正
        upper_body_rotation = upper_body_rotation * normal_correctqq
    else:
        # 横向きは減少補正
        upper_body_rotation = upper_body_rotation * decrease_correctqq

    bf.rotation = upper_body_rotation
    bone_frame_dic["上半身"].append(bf)
    
    # 下半身
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\xba\x94\xbc\x90\x67' # '下半身'
    direction = pos[0] - pos[7]
    up = QVector3D.crossProduct(direction, (pos[4] - pos[1]))
    lower_body_orientation = QQuaternion.fromDirection(direction, up)
    initial = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(0, 0, 1))
    bf.rotation = lower_body_orientation * initial.inverted() * normal_correctqq
    lower_body_rotation = bf.rotation
    bone_frame_dic["下半身"].append(bf)

    # 首
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8e\xf1' # '首'
    direction = pos[9] - pos[8]
    up = QVector3D.crossProduct((pos[14] - pos[11]), direction).normalized()
    neck_orientation = QQuaternion.fromDirection(up, direction)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, 0, -1), QVector3D(0, -1, 0))
    rotation = normal_correctqq * neck_orientation * initial_orientation.inverted()
    bf.rotation = upper_body_orientation.inverted() * rotation
    neck_rotation = bf.rotation
    bone_frame_dic["首"].append(bf)

    # 頭
    bf = VmdBoneFrame(frame)
    bf.name = b'\x93\xaa' # '頭'
    direction = pos[10] - pos[9]
    up = QVector3D.crossProduct((pos[9] - pos[8]), (pos[10] - pos[9]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, 1, 0), QVector3D(1, 0, 0))
    rotation = normal_correctqq * orientation * initial_orientation.inverted()
    bf.rotation = neck_rotation.inverted() * upper_body_rotation.inverted() * rotation
    bone_frame_dic["頭"].append(bf)
    
    # 左肩
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x8C\xA8' # '左肩'
    direction = pos[11] - pos[8]
    up = QVector3D.crossProduct((pos[11] - pos[8]), (pos[14] - pos[11]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(2, -0.8, 0), QVector3D(0.5, -0.5, -1))
    rotation = increase_correctqq * orientation * initial_orientation.inverted()
    # 左肩ポーンの回転から親ボーンの回転を差し引いてbf.rotationに格納する。
    # upper_body_rotation * bf.rotation = rotation なので、
    left_shoulder_rotation = upper_body_rotation.inverted() * rotation # 後で使うので保存しておく
    bf.rotation = left_shoulder_rotation
    bone_frame_dic["左肩"].append(bf)
    
    # 左腕
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x98\x72' # '左腕'
    direction = pos[12] - pos[11]
    up = QVector3D.crossProduct((pos[12] - pos[11]), (pos[13] - pos[12]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(1.73, -1, 0), QVector3D(1, 1.73, 0))
    rotation = increase_correctqq * orientation * initial_orientation.inverted()
    # 左腕ポーンの回転から親ボーンの回転を差し引いてbf.rotationに格納する。
    # upper_body_rotation * left_shoulder_rotation * bf.rotation = rotation なので、
    bf.rotation = left_shoulder_rotation.inverted() * upper_body_rotation.inverted() * rotation
    left_arm_rotation = bf.rotation # 後で使うので保存しておく
    bone_frame_dic["左腕"].append(bf)
    
    # 左ひじ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x82\xd0\x82\xb6' # '左ひじ'
    direction = pos[13] - pos[12]
    up = QVector3D.crossProduct((pos[12] - pos[11]), (pos[13] - pos[12]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(1.73, -1, 0), QVector3D(1, 1.73, 0))
    rotation = increase_correctqq * orientation * initial_orientation.inverted()
    # 左ひじポーンの回転から親ボーンの回転を差し引いてbf.rotationに格納する。
    # upper_body_rotation * left_shoulder_rotation * left_arm_rotation * bf.rotation = rotation なので、
    bf.rotation = left_arm_rotation.inverted() * left_shoulder_rotation.inverted() * upper_body_rotation.inverted() * rotation
    # bf.rotation = (upper_body_rotation * left_arm_rotation).inverted() * rotation # 別の表現
    bone_frame_dic["左ひじ"].append(bf)
    
    # 右肩
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x8C\xA8' # '右肩'
    direction = pos[14] - pos[8]
    up = QVector3D.crossProduct((pos[14] - pos[8]), (pos[11] - pos[14]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(-2, -0.8, 0), QVector3D(0.5, 0.5, 1))
    rotation = increase_correctqq * orientation * initial_orientation.inverted()
    # 右肩ポーンの回転から親ボーンの回転を差し引いてbf.rotationに格納する。
    # upper_body_rotation * bf.rotation = rotation なので、
    right_shoulder_rotation = upper_body_rotation.inverted() * rotation # 後で使うので保存しておく
    bf.rotation = right_shoulder_rotation
    bone_frame_dic["右肩"].append(bf)
    
    # 右腕
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x98\x72' # '右腕'
    direction = pos[15] - pos[14]
    up = QVector3D.crossProduct((pos[15] - pos[14]), (pos[16] - pos[15]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(-1.73, -1, 0), QVector3D(1, -1.73, 0))
    rotation = increase_correctqq * orientation * initial_orientation.inverted()
    bf.rotation = right_shoulder_rotation.inverted() * upper_body_rotation.inverted() * rotation
    right_arm_rotation = bf.rotation
    bone_frame_dic["右腕"].append(bf)
    
    # 右ひじ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x82\xd0\x82\xb6' # '右ひじ'
    direction = pos[16] - pos[15]
    up = QVector3D.crossProduct((pos[15] - pos[14]), (pos[16] - pos[15]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(-1.73, -1, 0), QVector3D(1, -1.73, 0))
    rotation = increase_correctqq * orientation * initial_orientation.inverted()
    bf.rotation = right_arm_rotation.inverted() * right_shoulder_rotation.inverted() * upper_body_rotation.inverted() * rotation
    bone_frame_dic["右ひじ"].append(bf)

    # 左足
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x91\xab' # '左足'
    direction = pos[5] - pos[4]
    up = QVector3D.crossProduct((pos[5] - pos[4]), (pos[6] - pos[5]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(-1, 0, 0))
    rotation = normal_correctqq * orientation * initial_orientation.inverted()
    bf.rotation = lower_body_rotation.inverted() * rotation
    left_leg_rotation = bf.rotation
    bone_frame_dic["左足"].append(bf)
    
    # 左ひざ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x82\xd0\x82\xb4' # '左ひざ'
    direction = pos[6] - pos[5]
    up = QVector3D.crossProduct((pos[5] - pos[4]), (pos[6] - pos[5]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(-1, 0, 0))
    rotation = normal_correctqq * orientation * initial_orientation.inverted()
    bf.rotation = left_leg_rotation.inverted() * lower_body_rotation.inverted() * rotation
    bone_frame_dic["左ひざ"].append(bf)

    # 右足
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x91\xab' # '右足'
    direction = pos[2] - pos[1]
    up = QVector3D.crossProduct((pos[2] - pos[1]), (pos[3] - pos[2]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(-1, 0, 0))
    rotation = normal_correctqq * orientation * initial_orientation.inverted()
    bf.rotation = lower_body_rotation.inverted() * rotation
    right_leg_rotation = bf.rotation
    bone_frame_dic["右足"].append(bf)
    
    # 右ひざ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x82\xd0\x82\xb4' # '右ひざ'
    direction = pos[3] - pos[2]
    up = QVector3D.crossProduct((pos[2] - pos[1]), (pos[3] - pos[2]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(-1, 0, 0))
    rotation = normal_correctqq * orientation * initial_orientation.inverted()
    bf.rotation = right_leg_rotation.inverted() * lower_body_rotation.inverted() * rotation
    bone_frame_dic["右ひざ"].append(bf)
    
    # センター(箱だけ作る)
    bf = VmdBoneFrame(frame)
    bf.name = b'\x83\x5A\x83\x93\x83\x5E\x81\x5B' # 'センター'
    bone_frame_dic["センター"].append(bf)
    
    # グルーブ(箱だけ作る)
    bf = VmdBoneFrame(frame)
    bf.name = b'\x83\x4F\x83\x8B\x81\x5B\x83\x75' # 'グルーブ'
    bone_frame_dic["グルーブ"].append(bf)

    # 左足ＩＫ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x91\xab\x82\x68\x82\x6a' # '左足ＩＫ'
    bone_frame_dic["左足ＩＫ"].append(bf)

    # 右足ＩＫ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x91\xab\x82\x68\x82\x6a' # '右足ＩＫ'
    bone_frame_dic["右足ＩＫ"].append(bf)


def read_positions(position_file):
    """Read joint position data"""
    f = open(position_file, "r")

    positions = []
    while True:
        line = f.readline()
        if not line:
            break
        line = line.rstrip('\r\n')
        a = re.split(' ', line)
        # 元データはz軸が垂直上向き。MMDに合わせるためにyとzを入れ替える。
        q = QVector3D(float(a[1]), float(a[3]), float(a[2])) # a[0]: index
        positions.append(q) # a[0]: index
    f.close()
    return positions

#複数フレームの読み込み
def read_positions_multi(position_file):
    """Read joint position data"""
    f = open(position_file, "r")

    positions = []
    while True:
        line = f.readline()
        if not line:
            break
        line = line.rstrip('\n')

        # 一旦カンマで複数行に分解
        inposition = []
        for inline in re.split(",\s*", line):
            if inline:
                # 1フレーム分に分解したら、空白で分解
                a = re.split(' ', inline)
                # print(a)
                # 元データはz軸が垂直上向き。MMDに合わせるためにyとzを入れ替える。
                q = QVector3D(float(a[1]), float(a[3]), float(a[2])) # a[0]: index
                inposition.append(q) # a[0]: index
        
        positions.append(inposition)
    f.close()
    return positions

def convert_position(pose_3d):
    positions = []
    for pose in pose_3d:
        for j in range(pose.shape[1]):
            q = QVector3D(pose[0, j], pose[2, j], pose[1, j])
            positions.append(q)
    return positions
    
# 関節位置情報のリストからVMDを生成します
def position_list_to_vmd_multi(positions_multi, vmd_file, smoothed_file, bone_csv_file, upright_idx, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, sdecimation, ddecimation):
    writer = VmdWriter()

    logger.info("角度計算開始")

    for frame, positions in enumerate(positions_multi):
        positions_to_frames(positions, frame, xangle)    

    logger.info("センター計算開始")

    # センターの計算
    calc_center(smoothed_file, bone_csv_file, positions_multi, upright_idx, center_xy_scale, center_z_scale)

    logger.info("IK計算開始")

    # IKの計算
    calc_IK(bone_csv_file)

    logger.info("グルーブ移管開始")

    # グルーブ移管
    set_groove(bone_csv_file)

    if mdecimation > 0 or idecimation > 0 or sdecimation > 0 or ddecimation > 0:
        logger.info("間引き開始")

        # 間引き
        for k,v in bone_frame_dic.items():
            decimate_bone_frames(k, mdecimation, idecimation, sdecimation, ddecimation)

    logger.info("VMD出力開始")

    # ディクショナリ型の疑似二次元配列から、一次元配列に変換
    bone_frames = []
    for k,v in bone_frame_dic.items():
        for bf in v:
            bone_frames.append(bf)

    # writer.write_vmd_file(vmd_file, bone_frames, showik_frames, expression_frames)
    writer.write_vmd_file(vmd_file, bone_frames)

def decimate_bone_frames(bone_name, mdecimation, idecimation, sdecimation, ddecimation):
    newbfs = []

    is_rot_hold = False
    is_pos_hold = False
    for n in range(len(bone_frame_dic[bone_name])):
        logger.debug("decimate_bone_frames n={0}".format(n))
        
        if n == 0:
            # 初回は問答無用
            newbfs.append(bone_frame_dic[bone_name][n])
        else:
            # 2つ前のフレーム
            if len(newbfs) < 2:
                prev2bf = newbfs[len(newbfs) - 1]
            else:
                prev2bf = newbfs[len(newbfs) - 2]

            # 1つ前のフレーム
            prev1bf = newbfs[len(newbfs) - 1]
            # 現在のフレーム
            nowbf = bone_frame_dic[bone_name][n]

            if (( bone_name == "左足ＩＫ" or bone_name == "右足ＩＫ" ) \
                and ( abs(Decimal(prev1bf.position.x()) - Decimal(nowbf.position.x())) <= idecimation / 10 \
                and abs(Decimal(prev1bf.position.y()) - Decimal(nowbf.position.y())) <= idecimation / 10 \
                and abs(Decimal(prev1bf.position.z()) - Decimal(nowbf.position.z())) <= idecimation / 10 ) ) :
                # 移動量がほとんどない場合、一時保留する
                is_pos_hold = True
            else:
                if is_pos_hold:
                    # 移動が動き始めた場合、保持FLG=ONの場合、前回のをコピーする
                    nowbf.position = prev1bf.position if is_pos_hold else nowbf.position
                    newbfs.append(nowbf)
                    is_pos_hold = False
                    continue

            if ( ( bone_name == "左足ＩＫ" or bone_name == "右足ＩＫ" ) \
                and (abs(Decimal(prev1bf.rotation.toEulerAngles().x()) - Decimal(nowbf.rotation.toEulerAngles().x())) <= 1 \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().y()) - Decimal(nowbf.rotation.toEulerAngles().y())) <= 1 \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().z()) - Decimal(nowbf.rotation.toEulerAngles().z())) <= 1)) :
                # 回転もがほとんどない場合、一時保留する
                is_rot_hold = True
            else:
                if is_rot_hold:
                    # 回転が動き始めた場合、保持FLG=ONの場合、前回のをコピーする
                    nowbf.rotation = prev1bf.rotation if is_rot_hold else nowbf.rotation
                    newbfs.append(nowbf)
                    is_rot_hold = False
                    continue
             
            # # 移動か回転が動いた場合、前回のを登録する
            # if is_pos_hold == False or is_rot_hold == False:
            #     newbfs.append(nowbf)
            #     continue

            # ○A-○B-○C　の移動パターン
            # ○A-○B の移動差分
            prev_pos_diff = prev2bf.position - prev1bf.position
            # ○B-○C の移動差分
            now_pos_diff = prev1bf.position - nowbf.position

            # logger.debug("prev2bf.position")
            # logger.debug(prev2bf.position)
            # logger.debug("prev1bf.position")
            # logger.debug(prev1bf.position)
            # logger.debug("nowbf.position")
            # logger.debug(nowbf.position)
            # logger.debug("prev_diff")
            # logger.debug(prev_pos_diff)
            # logger.debug("now_pos_diff")
            # logger.debug(now_pos_diff)

            # 前回と今回の移動差分の内積(マイナスの場合、鈍角で移動が大きい？)
            dot_pos_diff = QVector3D.dotProduct(prev_pos_diff, now_pos_diff)

            # logger.debug("dot_pos_diff")
            # logger.debug(dot_pos_diff)
            
            # 前回と今回の移動の回転量(グローブ用。Z軸回りに回転180すると、上下運動が反転する)
            diff_pos_rotation = QQuaternion.rotationTo(prev_pos_diff, now_pos_diff)

            # logger.debug("diff_pos_rotation")
            # logger.debug(diff_pos_rotation.toEulerAngles())

            # # ○A-○B の移動差分
            # prev_rot_diff = prev2bf.rotation - prev1bf.rotation
            # # ○B-○C の移動差分
            # now_rot_diff = prev1bf.rotation - nowbf.rotation

            # logger.debug("prev_rot_diff")
            # logger.debug(prev_rot_diff)
            # logger.debug(prev_rot_diff.toEulerAngles())
            # logger.debug("now_rot_diff")
            # logger.debug(now_rot_diff)
            # logger.debug(now_rot_diff.toEulerAngles())

            if bone_name == "センター" or bone_name == "グルーブ":
                if ((abs(Decimal(prev1bf.position.x()) - Decimal(nowbf.position.x())) > mdecimation) \
                    or (abs(Decimal(prev1bf.position.y()) - Decimal(nowbf.position.y())) > mdecimation) \
                    or (abs(Decimal(prev1bf.position.z()) - Decimal(nowbf.position.z())) > mdecimation) \
                    or (dot_pos_diff < 0) \
                    or (abs(diff_pos_rotation.toEulerAngles().z()) == 180)) :

                    # センター・グルーブの移動量が一定ならば追加
                    newbfs.append(nowbf)
                    continue
            
            if bone_name == "左足ＩＫ" or bone_name == "右足ＩＫ":
                if ((abs(Decimal(prev1bf.position.x()) - Decimal(nowbf.position.x())) > idecimation) \
                    or (abs(Decimal(prev1bf.position.y()) - Decimal(nowbf.position.y())) > idecimation) \
                    or (abs(Decimal(prev1bf.position.z()) - Decimal(nowbf.position.z())) > idecimation)) :

                    # 移動量が一定量が超えたら、登録
                    if is_pos_hold:
                        nowbf.position = prev1bf.position

                    # IKの移動量が一定ならば追加
                    newbfs.append(nowbf)
                    continue

            if ( ( abs(Decimal(prev1bf.rotation.toEulerAngles().x()) - Decimal(nowbf.rotation.toEulerAngles().x())) > sdecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().y()) - Decimal(nowbf.rotation.toEulerAngles().y())) <= ddecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().z()) - Decimal(nowbf.rotation.toEulerAngles().z())) <= ddecimation ) \
                or \
                ( abs(Decimal(prev1bf.rotation.toEulerAngles().x()) - Decimal(nowbf.rotation.toEulerAngles().x())) <= ddecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().y()) - Decimal(nowbf.rotation.toEulerAngles().y())) > sdecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().z()) - Decimal(nowbf.rotation.toEulerAngles().z())) <= ddecimation ) \
                or \
                ( abs(Decimal(prev1bf.rotation.toEulerAngles().x()) - Decimal(nowbf.rotation.toEulerAngles().x())) <= ddecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().y()) - Decimal(nowbf.rotation.toEulerAngles().y())) <= ddecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().z()) - Decimal(nowbf.rotation.toEulerAngles().z())) > sdecimation ) \
                or \
                ( abs(Decimal(prev1bf.rotation.toEulerAngles().x()) - Decimal(nowbf.rotation.toEulerAngles().x())) > ddecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().y()) - Decimal(nowbf.rotation.toEulerAngles().y())) > ddecimation \
                and abs(Decimal(prev1bf.rotation.toEulerAngles().z()) - Decimal(nowbf.rotation.toEulerAngles().z())) > ddecimation ) ) :

                # 同軸回転もしくは異軸回転が規定値を超えていた場合、登録
                newbfs.append(nowbf)

            # if n > 200:
            #     sys.exit()

    # 間引きしたフレームを登録しなおす
    bone_frame_dic[bone_name] = newbfs


# IKの計算
def calc_IK(bone_csv_file):
    logger.debug("bone_csv_file: "+ bone_csv_file)

    # ボーンファイルを開く
    with open(bone_csv_file, "r") as bf:
        reader = csv.reader(bf)

        for row in reader:

            if row[1] == "下半身":
                # 下半身ボーン
                lower_body_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左足":
                # 左足ボーン
                left_leg_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左ひざ":
                # 左ひざボーン
                left_knee_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左足首":
                # 左足首ボーン
                left_ankle_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左つま先":
                # 左つま先ボーン
                left_toes_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右足":
                # 右足ボーン
                right_leg_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右ひざ":
                # 右ひざボーン
                right_knee_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右足首":
                # 右足首ボーン
                right_ankle_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右つま先":
                # 右つま先ボーン
                right_toes_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[0] == "Bone" and row[1] == "左足ＩＫ":
                # 左足ＩＫボーン
                left_leg_ik_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[0] == "Bone" and row[1] == "右足ＩＫ":
                # 右足ＩＫボーン
                right_leg_ik_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "センター":
                # センターボーン
                center_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

    for n in range(len(bone_frame_dic["左足"])):
        logger.debug("足IK計算 frame={0}".format(n))

        # 左足IK
        (left_ankle_pos, left_ik_rotation, left_leg_diff_rotation) = \
            calc_IK_matrix(center_bone, lower_body_bone, left_leg_bone, left_knee_bone, left_ankle_bone, left_toes_bone, left_leg_ik_bone \
                , bone_frame_dic["センター"][n].position \
                , bone_frame_dic["下半身"][n].rotation, bone_frame_dic["左足"][n].rotation, bone_frame_dic["左ひざ"][n].rotation )

        # 右足IK
        (right_ankle_pos, right_ik_rotation, right_leg_diff_rotation) = \
            calc_IK_matrix(center_bone, lower_body_bone, right_leg_bone, right_knee_bone, right_ankle_bone, right_toes_bone, right_leg_ik_bone \
                , bone_frame_dic["センター"][n].position \
                , bone_frame_dic["下半身"][n].rotation, bone_frame_dic["右足"][n].rotation, bone_frame_dic["右ひざ"][n].rotation )

        # 両足IKがマイナスの場合
        if left_ankle_pos.y() < 0 and right_ankle_pos.y() < 0:
            ankle_pos_max = np.max([left_ankle_pos.y(), right_ankle_pos.y()])

            # センターも一緒にあげる
            left_ankle_pos.setY( left_ankle_pos.y() - ankle_pos_max )
            right_ankle_pos.setY( right_ankle_pos.y() - ankle_pos_max )
            bone_frame_dic["センター"][n].position.setY( bone_frame_dic["センター"][n].position.y() - ankle_pos_max )

        if left_ankle_pos.y() < 0 and right_ankle_pos.y() >= 0:
            # 左足だけの場合マイナス値は0に補正
            left_ankle_pos.setY(0)

        if right_ankle_pos.y() < 0 and left_ankle_pos.y() >= 0:
            # 右足だけの場合マイナス値は0に補正
            right_ankle_pos.setY(0)

        bone_frame_dic["左足ＩＫ"][n].position = left_ankle_pos
        bone_frame_dic["左足ＩＫ"][n].rotation = left_ik_rotation
        bone_frame_dic["左足"][n].rotation = left_leg_diff_rotation
        bone_frame_dic["左ひざ"][n].rotation = QQuaternion()

        bone_frame_dic["右足ＩＫ"][n].position = right_ankle_pos
        bone_frame_dic["右足ＩＫ"][n].rotation = right_ik_rotation
        bone_frame_dic["右足"][n].rotation = right_leg_diff_rotation
        bone_frame_dic["右ひざ"][n].rotation = QQuaternion()

        # if n >= 1015:
        #     sys.exit()



# 行列でIKの位置を求める
def calc_IK_matrix(center_bone, lower_body_bone, leg_bone, knee_bone, ankle_bone, toes_bone, ik_bone, center_pos, lower_body_rotation, leg_rotation, knee_rotation):

    # IKを求める ----------------------------

    # ローカル位置
    trans_vs = [0 for i in range(6)]
    # センターのローカル位置
    trans_vs[0] = center_bone + center_pos - ik_bone
    # 下半身のローカル位置
    trans_vs[1] = lower_body_bone - center_bone
    # 足のローカル位置
    trans_vs[2] = leg_bone - lower_body_bone
    # ひざのローカル位置 
    trans_vs[3] = knee_bone - leg_bone
    # 足首のローカル位置
    trans_vs[4] = ankle_bone - knee_bone
    # つま先のローカル位置
    trans_vs[5] = toes_bone - ankle_bone
    
    # 加算用クォータニオン
    add_qs = [0 for i in range(6)]
    # センターの回転
    add_qs[0] = QQuaternion()
    # 下半身の回転
    add_qs[1] = lower_body_rotation
    # 足の回転
    add_qs[2] = leg_rotation
    # ひざの回転
    add_qs[3] = knee_rotation
    # 足首の回転
    add_qs[4] = QQuaternion()
    # つま先の回転
    add_qs[5] = QQuaternion()

    # 行列
    matrixs = [0 for i in range(6)]

    for n in range(len(matrixs)):
        # 行列を生成
        matrixs[n] = QMatrix4x4()
        # 移動
        matrixs[n].translate(trans_vs[n])
        # 回転
        matrixs[n].rotate(add_qs[n])

        # logger.debug("matrixs[n] n={0}".format(n))
        # logger.debug(matrixs[n])

    # 足付け根の位置
    leg_pos = matrixs[0] * matrixs[1] * QVector4D(trans_vs[2], 1)

    # logger.debug("leg_pos")
    # logger.debug(leg_pos.toVector3D())

    # ひざの位置
    knee_pos = matrixs[0] * matrixs[1] * matrixs[2] * QVector4D(trans_vs[3], 1)

    # logger.debug("knee_pos")
    # logger.debug(knee_pos.toVector3D())

    # 足首の位置(行列の最後は掛けない)
    ankle_pos = matrixs[0] * matrixs[1] * matrixs[2] * matrixs[3] * QVector4D(trans_vs[4], 1)

    # logger.debug("ankle_pos")
    # logger.debug(ankle_pos.toVector3D())

    # つま先の位置
    toes_pos = matrixs[0] * matrixs[1] * matrixs[2] * matrixs[3] * matrixs[4] * QVector4D(trans_vs[5], 1)

    # logger.debug("toes_pos")
    # logger.debug(toes_pos.toVector3D())

    # 足首角度補正 ----------------------------------

    # 足付け根から足首までの距離
    ankle_leg_diff = ankle_pos - leg_pos

    # logger.debug("ankle_leg_diff")
    # logger.debug(ankle_leg_diff)
    # logger.debug(ankle_leg_diff.length())

    # ひざから足付け根までの距離
    knee_leg_diff = knee_bone - leg_bone

    # logger.debug("knee_leg_diff")
    # logger.debug(knee_leg_diff)
    # logger.debug(knee_leg_diff.length())

    # 足首からひざまでの距離
    ankle_knee_diff = ankle_bone - knee_bone

    # logger.debug("ankle_knee_diff")
    # logger.debug(ankle_knee_diff)
    # logger.debug(ankle_knee_diff.length())

    # つま先から足首までの距離
    toes_ankle_diff = toes_bone - ankle_bone

    # logger.debug("toes_ankle_diff")
    # logger.debug(toes_ankle_diff)
    # logger.debug(toes_ankle_diff.length())

    # 三辺から角度を求める

    # 足の角度
    leg_angle = calc_leg_angle(ankle_leg_diff, knee_leg_diff, ankle_knee_diff)
    # logger.debug("leg_angle:   {0}".format(leg_angle))

    # ひざの角度
    knee_angle = calc_leg_angle(knee_leg_diff, ankle_knee_diff, ankle_leg_diff)
    # logger.debug("knee_angle:  {0}".format(knee_angle))

    # 足首の角度
    ankle_angle = calc_leg_angle(ankle_knee_diff, ankle_leg_diff, knee_leg_diff)
    # logger.debug("ankle_angle: {0}".format(ankle_angle))

    # 足の付け根からひざへの方向を表す青い単位ベクトル(長さ1)
    # 足の付け根から足首へのベクトルをX軸回りに回転させる
    knee_v = QQuaternion.fromEulerAngles(leg_angle * -1, 0, 0) * ankle_leg_diff.toVector3D().normalized()

    # logger.debug("knee_v")
    # logger.debug(knee_v)

    ik_knee_3d = knee_v * knee_leg_diff.length() + leg_pos.toVector3D()

    # logger.debug("ik_knee_3d")
    # logger.debug(ik_knee_3d)

    # IKのひざ位置からFKのひざ位置に回転させる
    leg_diff_rotation = QQuaternion.rotationTo(knee_pos.toVector3D(), ik_knee_3d)

    # logger.debug("leg_diff_rotation")
    # logger.debug(leg_diff_rotation)
    # logger.debug(leg_diff_rotation.toEulerAngles())

    # FKと同じ状態の足首の向き
    ik_rotation = lower_body_rotation * leg_rotation * knee_rotation

    # logger.debug("ik_rotation")
    # logger.debug(ik_rotation)
    # logger.debug(ik_rotation.toEulerAngles())

    return (ankle_pos.toVector3D(), ik_rotation, leg_diff_rotation)

    

# 三辺から足の角度を求める
def calc_leg_angle(a, b, c):

    cos = ( pow(a.length(), 2) + pow(b.length(), 2) - pow(c.length(), 2) ) / ( 2 * a.length() * b.length() )

    # logger.debug("cos")
    # logger.debug(cos)

    radian = np.arccos(cos)

    # logger.debug("radian")
    # logger.debug(radian)

    angle = np.rad2deg(radian)

    # logger.debug("angle")
    # logger.debug(angle)

    return angle
    


# センターの計算
def calc_center(smoothed_file, bone_csv_file, positions_multi, upright_idx, center_xy_scale, center_z_scale):
    logger.debug("bone_csv_file: "+ bone_csv_file)

    # ボーンファイルを開く
    with open(bone_csv_file, "r") as bf:
        reader = csv.reader(bf)

        for row in reader:

            if row[1] == "首":
                # 首ボーン
                neck_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右足":
                # 右足ボーン
                right_leg_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左足":
                # 左足ボーン
                left_leg_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

    smoothed_2d = [[0 for i in range(3)] for j in range(len(bone_frame_dic["首"]))]
    n = 0
    with open(smoothed_file, "r") as sf:
        line = sf.readline() # 1行を文字列として読み込む(改行文字も含まれる)
        
        while line:
            # 空白で複数項目に分解
            smoothed = re.split("\s+", line)

            # logger.debug(smoothed)

            # 首の位置
            smoothed_2d[n][0] = QVector3D(float(smoothed[2]), float(smoothed[3]), 0)
            # 右足付け根
            smoothed_2d[n][1] = QVector3D(float(smoothed[16]), float(smoothed[17]), 0)
            # 左足付け根
            smoothed_2d[n][2] = QVector3D(float(smoothed[22]), float(smoothed[23]), 0)

            n += 1

            line = sf.readline()

    # logger.debug("neck_3d")
    # logger.debug(neck_3d)
    # logger.debug("right_leg_3d")
    # logger.debug(right_leg_3d)
    # logger.debug("left_leg_3d")
    # logger.debug(left_leg_3d)
    # logger.debug("center_3d")
    # logger.debug(center_3d)

    # ボーン頂点からの三角形面積
    bone_upright_area = calc_triangle_area(neck_3d, right_leg_3d, left_leg_3d)

    # logger.debug("smoothed_2d[upright_idx]")
    # logger.debug(smoothed_2d[upright_idx])

    # 直立フレームの三角形面積
    smoothed_upright_area = calc_triangle_area(smoothed_2d[upright_idx][0], smoothed_2d[upright_idx][1], smoothed_2d[upright_idx][2])

    # logger.debug("upright_area")
    # logger.debug(smoothed_upright_area)

    # ボーンと映像の三角形比率(スケール調整あり)
    upright_xy_scale = bone_upright_area / smoothed_upright_area * center_xy_scale

    # logger.debug("upright_scale")
    # logger.debug(upright_xy_scale)

    # 直立フレームの左足と右足の位置の平均
    upright_leg_avg = abs((smoothed_2d[upright_idx][1].y() + smoothed_2d[upright_idx][2].y()) / 2)

    # logger.debug("upright_leg_avg")
    # logger.debug(upright_leg_avg)

    # # 上半身から首までの距離
    # neck_upright_distance = upper_body_3d.distanceToPoint(neck_3d)

    # # logger.debug("neck_upright_distance")
    # # logger.debug(neck_upright_distance)

    # # 上半身から左足までの距離
    # left_leg_upright_distance = upper_body_3d.distanceToPoint(left_leg_3d)

    # # logger.debug("left_leg_upright_distance")
    # # logger.debug(left_leg_upright_distance)

    # # 上半身から左足までの距離
    # right_leg_upright_distance = upper_body_3d.distanceToPoint(right_leg_3d)
    
    # logger.debug("right_leg_upright_distance")
    # logger.debug(right_leg_upright_distance)

    # 3Dでの首・左足・右足の投影三角形
    pos_upright_area = calc_triangle_area(positions_multi[upright_idx][8], positions_multi[upright_idx][1], positions_multi[upright_idx][4])

    for n, smoothed in enumerate(smoothed_2d):
        logger.debug("センター計算 frame={0}".format(n))

        # 左足と右足の位置の平均
        leg_avg = abs((smoothed_2d[n][1].y() + smoothed_2d[n][2].y()) / 2)
        
        # 足の上下差
        leg_diff = upright_leg_avg - leg_avg

        # Y軸移動(とりあえずセンター固定)
        bone_frame_dic["センター"][n].position.setY(leg_diff * upright_xy_scale)
        
        # 首・左足・右足の中心部分をX軸移動
        x_avg = ((smoothed_2d[n][0].x() + smoothed_2d[n][1].x() + smoothed_2d[n][2].x()) / 3) \
                    - smoothed_2d[upright_idx][0].x()
        bone_frame_dic["センター"][n].position.setX(x_avg * upright_xy_scale)

        # 現在の映像の三角形面積
        # now_smoothed_area = calc_triangle_area(smoothed_2d[n][0], smoothed_2d[n][1], smoothed_2d[n][2])

        # logger.debug("smoothed_2d[n][0]")
        # logger.debug(smoothed_2d[n][0])

        # logger.debug("smoothed_2d[n][1]")
        # logger.debug(smoothed_2d[n][1])

        # logger.debug("smoothed_2d[n][2]")
        # logger.debug(smoothed_2d[n][2])

        # logger.debug("now_smoothed_area")
        # logger.debug(now_smoothed_area)

        # # 首の位置を上半身の傾きから求める
        # upper_slope = QQuaternion(0, 0, -1, 0).inverted() * bone_frame_dic["上半身"][n].rotation.normalized() * neck_upright_distance

        # # 左足の位置を下半身の傾きから求める
        # left_leg_slope = QQuaternion(0, 0.2, 1, 0).inverted() * bone_frame_dic["下半身"][n].rotation.normalized() * left_leg_upright_distance

        # # 右足の位置を下半身の傾きから求める
        # right_leg_slope = QQuaternion(0, -0.2, 1, 0).inverted() * bone_frame_dic["下半身"][n].rotation.normalized() * right_leg_upright_distance

        # # 現在のボーン構造の三角形面積
        # now_bone_area = calc_triangle_area(upper_slope.vector(), left_leg_slope.vector(), right_leg_slope.vector())
        
        # logger.debug("smoothed_upright_area")
        # logger.debug(smoothed_upright_area)

        # 3Dでの首・左足・右足の投影三角形
        pos_now_area = calc_triangle_area(positions_multi[n][8], positions_multi[n][1], positions_multi[n][4])

        # logger.debug("positions_multi[n][8]")
        # logger.debug(positions_multi[n][8])
        # logger.debug("positions_multi[n][1]")
        # logger.debug(positions_multi[n][1])
        # logger.debug("positions_multi[n][4]")
        # logger.debug(positions_multi[n][4])

        # logger.debug("pos_upright_area")
        # logger.debug(pos_upright_area)

        # logger.debug("pos_now_area")
        # logger.debug(pos_now_area)

        # 3Dでの現在の縮尺
        pos_scale = pos_now_area / pos_upright_area

        # logger.debug("pos_scale")
        # logger.debug(pos_scale)

        # logger.debug("pos_scale ** 2")
        # logger.debug(pos_scale ** 2)

        # 2Dでの首・左足・右足の投影三角形
        smoothed_now_area = calc_triangle_area(smoothed_2d[n][0], smoothed_2d[n][1], smoothed_2d[n][2])

        # logger.debug("smoothed_2d[n][0]")    
        # logger.debug(smoothed_2d[n][0])
        # logger.debug("smoothed_2d[n][1]")    
        # logger.debug(smoothed_2d[n][1])
        # logger.debug("smoothed_2d[n][2]")    
        # logger.debug(smoothed_2d[n][2])

        # logger.debug("smoothed_upright_area")
        # logger.debug(smoothed_upright_area)

        # logger.debug("smoothed_now_area")
        # logger.debug(smoothed_now_area)

        # 2Dでの現在の縮尺
        smoothed_scale = smoothed_now_area / smoothed_upright_area

        # logger.debug("smoothed_scale")
        # logger.debug(smoothed_scale)

        # logger.debug("((1 - smoothed_scale) ** 2)")
        # logger.debug(((1 - smoothed_scale) ** 2))

        # Z軸移動位置の算出
        now_z_scale = pos_scale * (1 - smoothed_scale)

        # logger.debug("now_z_scale")
        # logger.debug(now_z_scale)

        # logger.debug("now_z_scale * center_z_scale")
        # logger.debug(now_z_scale * center_z_scale)

        # Z軸の移動補正
        bone_frame_dic["センター"][n].position.setZ(now_z_scale * center_z_scale)


        # # 上半身の各軸傾き具合
        # rx = bone_frame_dic["上半身"][n].rotation.toEulerAngles().x()
        # ry = bone_frame_dic["上半身"][n].rotation.toEulerAngles().y() * -1
        # rz = bone_frame_dic["上半身"][n].rotation.toEulerAngles().z() * -1

        # # 傾いたところの頂点：首（傾きを反転させて正面向いた形にする）
        # smoothed_upright_slope_neck = calc_slope_point(smoothed_2d[upright_idx][8], rx * -1, ry * -1, rz * -1)
        # # 傾いたところの頂点：左足
        # smoothed_upright_slope_left_leg = calc_slope_point(smoothed_2d[upright_idx][1], rx * -1, ry * -1, rz * -1)
        # # 傾いたところの頂点：右足
        # smoothed_upright_slope_right_leg = calc_slope_point(smoothed_2d[upright_idx][2], rx * -1, ry * -1, rz * -1)

        # # 傾きを反転させた直立面積
        # smoothed_upright_slope_area = calc_triangle_area(smoothed_upright_slope_neck, smoothed_upright_slope_left_leg, smoothed_upright_slope_right_leg)

        # logger.debug("smoothed_upright_slope_area")
        # logger.debug(smoothed_upright_slope_area)

        # logger.debug("smoothed_upright_area")
        # logger.debug(smoothed_upright_area)

        # # 直立の関節の回転分面積を現在の関節面積で割って、大きさの比率を出す
        # now_z_scale = smoothed_upright_slope_area / smoothed_upright_area

        # if n == 340 or n == 341:

        #     logger.debug("smoothed_2d[upright_idx][0]")
        #     logger.debug(smoothed_2d[upright_idx][0])

        #     logger.debug("smoothed_2d[upright_idx][1]")
        #     logger.debug(smoothed_2d[upright_idx][1])

        #     logger.debug("smoothed_2d[upright_idx][2]")
        #     logger.debug(smoothed_2d[upright_idx][2])

        #     logger.debug("smoothed_upright_slope_neck")
        #     logger.debug(smoothed_upright_slope_neck)

        #     logger.debug("smoothed_upright_slope_left_leg")
        #     logger.debug(smoothed_upright_slope_left_leg)

        #     logger.debug("smoothed_upright_slope_right_leg")
        #     logger.debug(smoothed_upright_slope_right_leg)

        #     logger.debug("smoothed_upright_slope_area")
        #     logger.debug(smoothed_upright_slope_area)

        # # 傾きの総数 - 各傾きの絶対値＝傾き具合
        # rsum = (90 - abs(rx)) + (90 - abs(90 - abs(ry))) + (90 - abs(rz))
        # # 360で割って、どれくらい傾いているか係数算出(1に近いほど正面向き)
        # rsum_scale = (180 / rsum) ** ( center_z_scale ** center_z_scale)

        # logger.debug("rx")
        # logger.debug(rx)

        # logger.debug("ry")
        # logger.debug(ry)

        # logger.debug("rz")
        # logger.debug(rz)

        # logger.debug("rsum")
        # logger.debug(rsum)

        # logger.debug("rsum_scale")
        # logger.debug(rsum_scale)

        # logger.debug("now_z_scale")
        # logger.debug(now_z_scale)
            
        # # 1より大きい場合、近くにある(マイナス)
        # # 1より小さい場合、遠くにある(プラス)
        # now_z_scale_pm = 1 - now_z_scale

        # logger.debug("now_z_scale_pm")
        # logger.debug(now_z_scale_pm)

        # logger.debug("now_z_scale_pm * rsum_scale")
        # logger.debug(now_z_scale_pm * rsum_scale)

        # if n < 20:
        # logger.debug("upper_slope")
        # logger.debug(upper_slope)
        # logger.debug(upper_slope.vector())

        # logger.debug("left_leg_slope")
        # logger.debug(left_leg_slope)
        # logger.debug(left_leg_slope.vector())

        # logger.debug("right_leg_slope")
        # logger.debug(right_leg_slope)
        # logger.debug(right_leg_slope.vector())

        # logger.debug("bone_upright_area")
        # logger.debug(bone_upright_area)

        # logger.debug("now_bone_area")
        # logger.debug(now_bone_area)

        # logger.debug("now_scale")
        # logger.debug(now_scale)

        # logger.debug("now_scale * center_scale")
        # logger.debug(now_scale * center_scale)

        # logger.debug("leg_avg")
        # logger.debug(leg_avg)
        
        # logger.debug("leg_diff")
        # logger.debug(leg_diff)
        
        # logger.debug("smoothed_2d[upright_idx][0].x()")
        # logger.debug(smoothed_2d[upright_idx][0].x())
        
        # logger.debug("smoothed_2d[n][0].x()")
        # logger.debug(smoothed_2d[n][0].x())
        
        # logger.debug("smoothed_2d[n][1].x()")
        # logger.debug(smoothed_2d[n][1].x())
        
        # logger.debug("smoothed_2d[n][2].x()")
        # logger.debug(smoothed_2d[n][2].x())
        
        # logger.debug("((smoothed_2d[n][0].x() + smoothed_2d[n][1].x() + smoothed_2d[n][2].x()) / 3)")
        # logger.debug(((smoothed_2d[n][0].x() + smoothed_2d[n][1].x() + smoothed_2d[n][2].x()) / 3))
        
        # logger.debug("x_avg")
        # logger.debug(x_avg)

        # logger.debug("now_smoothed_area")
        # logger.debug(now_smoothed_area)

        # logger.debug("now_position_area")
        # logger.debug(now_position_area)

        # logger.debug("now_scale")
        # logger.debug(now_scale)

        # logger.debug("now_scale * upright_position_scale")
        # logger.debug(now_scale * upright_position_scale)

        

        # # モデルの上半身の傾き
        # upper_body_euler = QVector3D(
        #     bone_frame_dic["上半身"][n].rotation.toEulerAngles().x() \
        #     , bone_frame_dic["上半身"][n].rotation.toEulerAngles().y() * -1 \
        #     , bone_frame_dic["上半身"][n].rotation.toEulerAngles().z() * -1 \
        # ) 



        # # モデルの上半身の傾き。初期位置からフレームの位置まで回転
        # upper_body_qq = QQuaternion(0, upper_body_3d)
        # # upper_body_qq = QQuaternion.rotationTo( upper_body_3d, bone_frame_dic["上半身"][n].rotation.vector() )

        # if n < 20:
        #     logger.debug("bone_frame_dic")
        #     logger.debug(bone_frame_dic["上半身"][n].rotation)
        #     logger.debug(bone_frame_dic["上半身"][n].rotation.toVector4D())
        #     logger.debug(bone_frame_dic["上半身"][n].rotation.toEulerAngles())
        #     v = bone_frame_dic["上半身"][n].rotation.toVector4D()
        #     logger.debug(v.x() * v.w())
        #     logger.debug(v.y() * v.w() * -1)
        #     logger.debug(v.z() * v.w() * -1)
        #     logger.debug("upper_body_qq")
        #     logger.debug(upper_body_qq)
        #     logger.debug(upper_body_qq.toEulerAngles())
        #     logger.debug(upper_body_qq.toVector4D())
        
# センターY軸をグルーブY軸に移管
def set_groove(bone_csv_file):

    # グルーブボーンがあるか
    is_groove = False
    # ボーンファイルを開く
    with open(bone_csv_file, "r") as bf:
        reader = csv.reader(bf)

        for row in reader:
            if row[1] == "グルーブ":
                is_groove = True
                break

    if is_groove:

        for n in range(len(bone_frame_dic["センター"])):
            # logger.debug("グルーブ移管 frame={0}".format(n))

            # グルーブがある場合、Y軸をグルーブに設定
            bone_frame_dic["グルーブ"][n].position = QVector3D(0, bone_frame_dic["センター"][n].position.y(), 0)
            bone_frame_dic["センター"][n].position = QVector3D(bone_frame_dic["センター"][n].position.x(), 0, bone_frame_dic["センター"][n].position.z())


# 直立姿勢から傾いたところの頂点を求める
# FIXME クォータニオンで求められないか要調査
def calc_slope_point(upright, rx, ry, rz):
    # // ｘ軸回転
    # x1 = dat[n][0] ;
    # y1 = dat[n][1]*cos(rx)-dat[n][2]*sin(rx) ;
    # z1 = dat[n][1]*sin(rx)+dat[n][2]*cos(rx) ;
    x1 = upright.x()
    y1 = upright.y() * np.cos(np.radians(rx)) - upright.z() * np.sin(np.radians(rx))
    z1 = upright.y() * np.sin(np.radians(rx)) + upright.z() * np.cos(np.radians(rx))

    # // ｙ軸回転
    # x2 = x1*cos(ry)+z1*sin(ry) ;
    # y2 = y1 ;
    # z2 = z1*cos(ry)-x1*sin(ry) ;
    x2 = x1 * np.cos(np.radians(ry)) + z1 * np.sin(np.radians(ry))
    y2 = y1
    z2 = z1 * np.cos(np.radians(ry)) - x1 * np.sin(np.radians(ry))

    # // ｚ軸回転
    # x3 = x2*cos(rz)-y2*sin(rz) ;
    # y3 = x2*sin(rz)+y2*cos(rz) ;
    # z3 = z2 ;
    x3 = x2 * np.cos(np.radians(rz)) - y2 * np.sin(np.radians(rz))
    y3 = x2 * np.sin(np.radians(rz)) + y2 * np.cos(np.radians(rz))
    z3 = z2

    return QVector3D(x3, y3, z3)

# 3つの頂点から三角形の面積を計算する
def calc_triangle_area(a, b, c):
    # logger.debug(a)
    # logger.debug(b)
    # logger.debug(c)
    # logger.debug("(a.y() - c.y())")
    # logger.debug((a.y() - c.y()))
    # logger.debug("(b.x() - c.x())")
    # logger.debug((b.x() - c.x()))
    # logger.debug("(b.y() - c.y())")
    # logger.debug((b.y() - c.y()))
    # logger.debug("(c.x() - a.x())")
    # logger.debug((c.x() - a.x()))
    return abs(( ((a.y() - c.y()) * (b.x() - c.x())) \
                    + ((b.y() - c.y()) * (c.x() - a.x())) ) / 2 )
    

def position_multi_file_to_vmd(position_file, vmd_file, smoothed_file, bone_csv_file, upright_idx, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, sdecimation, ddecimation):
    positions_multi = read_positions_multi(position_file)
    position_list_to_vmd_multi(positions_multi, vmd_file, smoothed_file, bone_csv_file, upright_idx, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, sdecimation, ddecimation)
    
if __name__ == '__main__':
    import sys
    if (len(sys.argv) < 2):
        usage(sys.argv[0])

    parser = argparse.ArgumentParser(description='3d-pose-baseline to vmd')
    parser.add_argument('-t', '--target', dest='target', type=str,
                        help='target directory')
    parser.add_argument('-b', '--bone', dest='bone', type=str,
                        help='target model bone csv')
    parser.add_argument('-v', '--verbose', dest='verbose', type=int,
                        default=2,
                        help='logging level')
    parser.add_argument('-u', '--upright-frame', dest='upright', type=int,
                        default=0,
                        help='upright frame index')
    parser.add_argument('-c', '--center-xyscale', dest='centerxy', type=int,
                        default=0,
                        help='center scale')
    parser.add_argument('-z', '--center-z-scale', dest='centerz', type=float,
                        default=0,
                        help='center z scale')
    parser.add_argument('-x', '--x-angle', dest='xangle', type=int,
                        default=0,
                        help='global x angle correction')
    parser.add_argument('-s', '--same-born-decimation', dest='sdecimation', type=int,
                        default=0,
                        help='born frame same decimation angle')
    parser.add_argument('-d', '--difference-born-decimation', dest='ddecimation', type=int,
                        default=0,
                        help='born frame difference decimation angle')
    parser.add_argument('-m', '--center-move-born-decimation', dest='mdecimation', type=float,
                        default=0,
                        help='born frame center decimation move')
    parser.add_argument('-i', '--ik-move-born-decimation', dest='idecimation', type=float,
                        default=0,
                        help='born frame ik decimation move')
    args = parser.parse_args()

    # resultディレクトリだけ指定させる
    base_dir = args.target

    # 入力と出力のファイル名は固定
    position_file = base_dir + "/pos.txt"
    smoothed_file = base_dir + "/smoothed.txt"
    vmd_file = "{0}/output_{1:%Y%m%d_%H%M%S}.vmd".format(base_dir, datetime.datetime.now())

    level = {0:logging.ERROR,
             1:logging.WARNING,
             2:logging.INFO,
             3:logging.DEBUG}

    # ログレベル設定
    logger.setLevel(level[args.verbose])

    # if os.path.exists('predictor/shape_predictor_68_face_landmarks.dat'):
    #     head_rotation = 

    position_multi_file_to_vmd(position_file, vmd_file, smoothed_file, args.bone, args.upright - 1, args.centerxy, args.centerz, args.xangle, args.mdecimation, args.idecimation, args.sdecimation, args.ddecimation)

    logger.info("VMDファイル出力完了: {0}".format(vmd_file))
